// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.time.Instant;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.flywheelSysIDCommand;
import frc.robot.Commands.setFlywheelTest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Commands.setIntakeAction;
import frc.robot.Commands.toggleIntakeStorage;
import frc.robot.Commands.AutoCommands.moveTurretAuto;
import frc.robot.Commands.AutoCommands.setFlywheelAuto;
import frc.robot.Commands.AutoCommands.setFlywheelSlowAuto;
import frc.robot.Commands.AutoCommands.setIntakeActionAuto;
import frc.robot.Commands.AutoCommands.setIntakeStorageAuto;
import frc.robot.Commands.AutoCommands.setSpindexerAuto;
import frc.robot.Commands.ResetCommands.antijam;
import frc.robot.Commands.ResetCommands.resetHoodEncoder;
import frc.robot.Commands.ResetCommands.resetSliderEncoder;
import frc.robot.Commands.ResetCommands.resetTurretEncoder;
import frc.robot.Commands.TeleopCommands.FlywheelTeleop;
import frc.robot.Commands.TeleopCommands.SpindexerTeleop;
import frc.robot.Commands.TeleopCommands.SwerveTeleop;
import frc.robot.Commands.TeleopCommands.TurretTeleop;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
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
  private final SpindexerSubsystem mSpindexer = new SpindexerSubsystem();
  private final VisionSubsystem mVisionSubsystem;
  private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mOperator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final TurretSubsystem mTurret = new TurretSubsystem();
  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final FlywheelSubsystem mFlywheel = new FlywheelSubsystem();
  private final SendableChooser<PathPlannerAuto> autoChooser;
  
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
    NamedCommands.registerCommand("FlywheelOnSlow", new setFlywheelSlowAuto(mFlywheel));
    NamedCommands.registerCommand("FlywheelOff", new InstantCommand(() -> {
      mFlywheel.setMode(FlywheelMode.OFF, 0.0);
      mFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
    }));
    NamedCommands.registerCommand("IntakeOn", new setIntakeActionAuto(mIntake, IntakeActionMode.INTAKE));
    NamedCommands.registerCommand("IntakeOff", new setIntakeActionAuto(mIntake, IntakeActionMode.OFF));

    NamedCommands.registerCommand("SOTM", new ParallelCommandGroup(getTurretCommand(), getFlywheelCommand()));
    NamedCommands.registerCommand("SpindexerSOTM", getSpindexerCommand());

    autoChooser = new SendableChooser<PathPlannerAuto>();
    autoChooser.addOption("Preload Right Auto", new PathPlannerAuto("Preload Right Auto"));
    autoChooser.addOption("Preload Left Auto", new PathPlannerAuto("Preload Left Auto"));
    autoChooser.setDefaultOption("Preload Center Auto", new PathPlannerAuto("Preload Auto"));
    autoChooser.addOption("Left Neutral Repeat", new PathPlannerAuto("Right Neutral Repeat",true));
    autoChooser.addOption("Right Neutral Repeat", new PathPlannerAuto("Right Neutral Repeat"));
    autoChooser.addOption("Experimental Left Neutral", new PathPlannerAuto("Experimental Double Cycle"));
    autoChooser.addOption("Experimental Right Neutral", new PathPlannerAuto("Experimental Double Cycle", true));
    autoChooser.addOption("Depot SOTM Zone", new PathPlannerAuto("Depo zone"));
    autoChooser.addOption("Depot Safe Zone", new PathPlannerAuto("Depot Safe"));
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
      mFlywheel.setMode(FlywheelMode.OFF);
    }));

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

    mOperator.rightBumper().whileTrue(new antijam(mSpindexer, mFlywheel));
    mOperator.rightBumper().onFalse(new InstantCommand(() -> {
      mFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
      mFlywheel.setMode(FlywheelMode.OFF);
    }).alongWith(new SpindexerTeleop(mSpindexer, SpindexerMode.OFF, () -> true, () -> FieldMathHelpers.Location.ALLIANCE_ZONE)));

    mOperator.x().onTrue(new InstantCommand(() -> mSwerve.resetOdometry(mVisionSubsystem.getLastValidPose())));
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  // Autonomous commands
  public Command getTurretCommand(){
    return new TurretTeleop(
      mTurret, 
      mSwerve::getHeadingDegrees,
      mSwerve::getPose,
      mSwerve::getFieldRelativeSpeeds,
      mSwerve::getAngularVelocity,
      mSwerve::getLocation);
  }

  public Command getFlywheelCommand(){
    return new FlywheelTeleop(mFlywheel,
      mSwerve::getPose,
      mSwerve::getFieldRelativeSpeeds,
      mSwerve::getAngularVelocity,
      mSwerve::getLocation);
  }

  public Command getSpindexerCommand(){
    return new SpindexerTeleop(
        mSpindexer, 
        SpindexerMode.LOAD,
        () -> mTurret.getTurretAtSetpoint() && mFlywheel.getFlywheelAtTarget(),
        mSwerve::getLocation); 
  }
}
