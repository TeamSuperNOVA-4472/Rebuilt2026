// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import javax.tools.JavaFileManager.Location;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.autoAlignToClimb;
import frc.robot.Commands.flywheelSysIDCommand;
import frc.robot.Commands.setClimb;
import frc.robot.Commands.setFlywheelTest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Commands.setIntakeAction;
import frc.robot.Commands.toggleIntakeStorage;
import frc.robot.Commands.AutoCommands.moveTurretAuto;
import frc.robot.Commands.AutoCommands.setClimbAuto;
import frc.robot.Commands.AutoCommands.setFlywheelAuto;
import frc.robot.Commands.AutoCommands.setFlywheelSlowAuto;
import frc.robot.Commands.AutoCommands.setIntakeActionAuto;
import frc.robot.Commands.AutoCommands.setIntakeStorageAuto;
import frc.robot.Commands.AutoCommands.setSpindexerAuto;
import frc.robot.Commands.Autos.ShootPreloadFromStandstill;
import frc.robot.Commands.ResetCommands.resetHoodEncoder;
import frc.robot.Commands.ResetCommands.resetSliderEncoder;
import frc.robot.Commands.ResetCommands.resetTurretEncoder;
import frc.robot.Commands.TeleopCommands.FlywheelTeleop;
import frc.robot.Commands.TeleopCommands.SpindexerTeleop;
import frc.robot.Commands.TeleopCommands.SwerveTeleop;
import frc.robot.Commands.TeleopCommands.TurretTeleop;
import frc.robot.Commands.autoAlignToClimb.ClimbDirection;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.ClimbSubsystem.ClimbState;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;
import frc.robot.Subsystems.IntakeSubsystem.IntakeActionMode;
import frc.robot.Subsystems.IntakeSubsystem.IntakeStorageMode;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;


public class RobotContainer {
  private final IntakeSubsystem mIntake = new IntakeSubsystem();
  private final ClimbSubsystem mClimb = new ClimbSubsystem();
  private final SpindexerSubsystem mSpindexer = new SpindexerSubsystem();
  private final VisionSubsystem mVisionSubsystem;
  private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mOperator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final TurretSubsystem mTurret = new TurretSubsystem();
  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final FlywheelSubsystem mFlywheel = new FlywheelSubsystem();
  private final PowerDistribution mPdh = new PowerDistribution(1, ModuleType.kRev);
  private final SendableChooser<Command> autoChooser;
  
  private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);
  private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);
  private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);

  private final SwerveTeleop mSwerveTeleop = new SwerveTeleop(
    () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())), 
    () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
    () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
    () -> mDriver.a().getAsBoolean(),
    () -> mDriver.getRightTriggerAxis(),
    mSwerve);
  
  private final TurretTeleop mMoveTurretAbsolute = new TurretTeleop(
      mTurret, 
      mSwerve::getHeadingDegrees,
      mSwerve::getPose,
      mSwerve::getFieldRelativeSpeeds,
      mSwerve::getAngularVelocity,
      mSwerve::getLocation);

  private final resetTurretEncoder mResetTurretEncoder = new resetTurretEncoder(mTurret);

  public Command getTurretReset()
  {
    return mResetTurretEncoder;
  }

  public RobotContainer() {
    // Defaults for swerve and turret
    mSwerve.setDefaultCommand(mSwerveTeleop);
    mTurret.setDefaultCommand(mMoveTurretAbsolute);

    mTurret.setAngularSpeedSupplier(mSwerve::getAngularVelocity);

    mVisionSubsystem = new VisionSubsystem(mSwerve::getHeadingDegrees, mSwerve::getAngularVelocity,
    (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
      mSwerve.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
    });

    NamedCommands.registerCommand("IntakeOut", new setIntakeStorageAuto(mIntake, IntakeStorageMode.OUT));
    NamedCommands.registerCommand("IntakeStore", new setIntakeStorageAuto(mIntake, IntakeStorageMode.STORED));
    NamedCommands.registerCommand("ResetTurretEncoder", new resetTurretEncoder(mTurret));
    NamedCommands.registerCommand("AimAtHub", new moveTurretAuto(
      mTurret, 
      mSwerve::getHeadingDegrees,
      () -> FieldMathHelpers.getRotationToPassOrShootWithSomeSpeed(
        mSwerve.getPose(),
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
        mSwerve.getAngularVelocity())));

    NamedCommands.registerCommand("SpindexerOff", new setSpindexerAuto(mSpindexer, SpindexerMode.OFF));
    NamedCommands.registerCommand("SpindexerOn", new setSpindexerAuto(mSpindexer, SpindexerMode.LOAD));
    NamedCommands.registerCommand("FlywheelOn", new setFlywheelAuto(mFlywheel, () -> FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(
      mSwerve.getPose(), 
      mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
      mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
      mSwerve.getAngularVelocity()
      ).getNorm()));
    NamedCommands.registerCommand("FlywheelOnSlow", new setFlywheelSlowAuto(mFlywheel, () -> FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(
      mSwerve.getPose(), 
      mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
      mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
      mSwerve.getAngularVelocity()
      ).getNorm()));
    NamedCommands.registerCommand("FlywheelOff", new InstantCommand(() -> {
      mFlywheel.setMode(FlywheelMode.OFF, 0.0);
      mFlywheel.setHoodTarget(20.0);
    }));
    NamedCommands.registerCommand("IntakeOn", new setIntakeActionAuto(mIntake, IntakeActionMode.INTAKE));
    NamedCommands.registerCommand("IntakeOff", new setIntakeActionAuto(mIntake, IntakeActionMode.OFF));
    NamedCommands.registerCommand("ClimbStored", new setClimbAuto(mClimb,ClimbState.STORED));
    NamedCommands.registerCommand("ClimbUp", new setClimbAuto(mClimb,ClimbState.UP));
    NamedCommands.registerCommand("ClimbClimb", new setClimbAuto(mClimb,ClimbState.CLIMB));

    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Preload Right Auto", new PathPlannerAuto("Preload Right Auto"));
    autoChooser.addOption("Preload Left Auto", new PathPlannerAuto("Preload Left Auto"));
    //autoChooser.addOption("Shoot Preload From Standstill", new ShootPreloadFromStandstill(mFlywheel, mSwerve, mSpindexer));
    //autoChooser.addOption("Depot From Center", new PathPlannerAuto("Depo zone"));
    //autoChooser.addOption("Right Neutral Zone", new PathPlannerAuto("Neutral zone right side"));
    //autoChooser.addOption("Left Neutral Zone", new PathPlannerAuto("Neutral zone agressive"));
    autoChooser.setDefaultOption("Preload Center Auto", new PathPlannerAuto("Preload Auto"));
    //autoChooser.addOption("Path 2 Auto", new PathPlannerAuto("Path 2 Auto"));
    autoChooser.addOption("Right neutral repeat", new PathPlannerAuto("Right neutral repeat"));
    autoChooser.addOption("left neutral repeat Auto", new PathPlannerAuto("left neutral repeat Auto"));
    autoChooser.addOption("depo and climb auto", new PathPlannerAuto("depo and climb auto"));
    autoChooser.addOption("Right neutral climb Auto", new PathPlannerAuto("Right neutral climb Auto"));
    SmartDashboard.putData("Auto Selector", autoChooser);

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // Spindexer Bindings
    mDriver.leftBumper().whileTrue(
      new SpindexerTeleop(
        mSpindexer, 
        SpindexerMode.LOAD,
        () -> mTurret.getTurretAtSetpoint() && mFlywheel.getFlywheelAtTarget(),
        mSwerve::getLocation));

    mDriver.leftBumper().onFalse(
      new SpindexerTeleop(mSpindexer, SpindexerMode.OFF, () -> true, () -> FieldMathHelpers.Location.ALLIANCE_ZONE)
    );

    // Flywheel and Hood Bindings
    mDriver.leftTrigger(OperatorConstants.kTriggerThreshold).whileTrue(
      new FlywheelTeleop(mFlywheel,
      mSwerve::getPose,
      mSwerve::getFieldRelativeSpeeds,
      mSwerve::getAngularVelocity,
      mSwerve::getLocation));

    mDriver.leftTrigger(OperatorConstants.kTriggerThreshold).onFalse(new InstantCommand(() -> {
      mFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
      mFlywheel.setMode(FlywheelMode.OFF, 0);
    }));

    //mDriver.leftTrigger(OperatorConstants.kTriggerThreshold).whileTrue(new setFlywheelTest(mFlywheel, mDriver.povUp()::getAsBoolean, mDriver.povDown()::getAsBoolean, mDriver.povRight()::getAsBoolean, mDriver.povLeft()::getAsBoolean));

    //mDriver.x().whileTrue(new autoAlignToClimb(mSwerve, mClimb, ClimbDirection.LEFT));
  }

  private void configureOperatorBindings()
  {

    // Intake Action Bindings
    mOperator.leftBumper().onTrue(new setIntakeAction(mIntake, IntakeActionMode.OUTTAKE));
    mOperator.leftTrigger(OperatorConstants.kTriggerThreshold).onTrue(new setIntakeAction(mIntake, IntakeActionMode.INTAKE));
    mOperator.leftBumper().or(mOperator.leftTrigger(OperatorConstants.kTriggerThreshold)).onFalse(new setIntakeAction(mIntake, IntakeActionMode.OFF));

    // Intake pump
    mOperator.rightTrigger(Constants.OperatorConstants.kTriggerThreshold).onTrue(new toggleIntakeStorage(mIntake));

    // Encoder resets
    mOperator.povDown().onTrue(new resetHoodEncoder(mFlywheel));

    mOperator.povUp().onTrue(new resetSliderEncoder(mIntake));

    mOperator.povRight().onTrue(new resetTurretEncoder(mTurret));

    // TODO: make this a constant
    mOperator.y().onTrue(new setClimb(mClimb, ClimbState.UP));
    mOperator.a().onTrue(new setClimb(mClimb, ClimbState.CLIMB));
    mOperator.b().onTrue(new setClimb(mClimb, ClimbState.STORED));

    mOperator.x().onTrue(new InstantCommand(() -> mSwerve.resetOdometry(mVisionSubsystem.getLastValidPose())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void getAmperageToLog(){
    SmartDashboard.putNumber("Total Amperage", mPdh.getTotalCurrent());
  }
}
