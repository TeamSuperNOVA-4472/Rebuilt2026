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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.SwerveTeleop;
import frc.robot.Commands.flywheelSysIDCommand;
import frc.robot.Commands.setFlywheel;
import frc.robot.Commands.setFlywheelTest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Commands.moveTurretAbsolute;
import frc.robot.Commands.setIntakeAction;
import frc.robot.Commands.setSpindexer;
import frc.robot.Commands.toggleIntakeStorage;
import frc.robot.Commands.Autos.ShootPreloadFromStandstill;
import frc.robot.Commands.ResetCommands.resetHoodEncoder;
import frc.robot.Commands.ResetCommands.resetSliderEncoder;
import frc.robot.Commands.ResetCommands.resetTurretEncoder;
import frc.robot.Commands.SafeCommands.moveTurretSafe;
import frc.robot.Commands.SafeCommands.setFlywheelSafe;
import frc.robot.Subsystems.ClimbSubsystem;
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
  private final ClimbSubsystem mClimb = new ClimbSubsystem();
  private final SpindexerSubsystem mSpindexer = new SpindexerSubsystem();
  private final VisionSubsystem mVisionSubsystem;
  private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mOperator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final TurretSubsystem mTurret = new TurretSubsystem();
  private final SwerveSubsystem mSwerve = new SwerveSubsystem();
  private final FlywheelSubsystem mFlywheel = new FlywheelSubsystem();
  private final SendableChooser<Command> autoChooser;
  
  private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);
  private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);
  private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(OperatorConstants.kSlewLimit);

  private final SwerveTeleop mSwerveTeleop = new SwerveTeleop(
    () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())), 
    () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
    () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
    () -> mDriver.a().getAsBoolean(),
    mSwerve);
  
  private final moveTurretAbsolute mMoveTurretAbsolute = new moveTurretAbsolute(
      mTurret, 
      mSwerve::getHeadingDegrees,
      () -> FieldMathHelpers.getRotationToPassOrShootWithSomeSpeed(
        mSwerve.getPose(),
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
        mSwerve.getAngularVelocity()));

  private final resetTurretEncoder mResetTurretEncoder = new resetTurretEncoder(mTurret);

  public Command getTurretReset()
  {
    return mResetTurretEncoder;
  }

  public RobotContainer() {
    // Defaults for swerve and turret
    mSwerve.setDefaultCommand(mSwerveTeleop);
    mTurret.setDefaultCommand(mMoveTurretAbsolute);

    Trigger safeModeOn = new Trigger(mTurret::getSafeModeEnabled);
    safeModeOn.whileTrue(new moveTurretSafe(mTurret));

    mTurret.setAngularSpeedSupplier(mSwerve::getAngularVelocity);
    mTurret.setAngularAccelerationSupplier(mSwerve::getAngularAcceleration);

    mVisionSubsystem = new VisionSubsystem(mSwerve::getHeadingDegrees, mSwerve::getAngularVelocity,
    (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
      mSwerve.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
    });

    // Only debounces (waits for the condition to be true or false for a bit) on the way down from the bump
    Trigger onBump = new Trigger(() -> mSwerve.getLocation() == FieldMathHelpers.Location.BUMP).debounce(VisionConstants.kCooldownBump, DebounceType.kFalling);
    onBump.onTrue(new InstantCommand(mVisionSubsystem::disableMT2));
    onBump.onFalse(new InstantCommand(() -> {
      mVisionSubsystem.enableMT2();
    }));

    NamedCommands.registerCommand("ToggleIntakeStore", new toggleIntakeStorage(mIntake));
    NamedCommands.registerCommand("ResetTurretEncoder", new resetTurretEncoder(mTurret));
    NamedCommands.registerCommand("FollowWithTurret", new moveTurretAbsolute(
      mTurret, 
      mSwerve::getHeadingDegrees,
      () -> FieldMathHelpers.getRotationToPassOrShootWithSomeSpeed(
        mSwerve.getPose(),
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
        mSwerve.getAngularVelocity())));

    NamedCommands.registerCommand("SpindexerOff", new InstantCommand(() -> mSpindexer.setMode(SpindexerMode.OFF)));
    NamedCommands.registerCommand("SpindexerOn", new InstantCommand(() -> mSpindexer.setMode(SpindexerMode.LOAD)));
    NamedCommands.registerCommand("FlywheelOn", new setFlywheel(mFlywheel, () -> FieldMathHelpers.getTranslationToHub(mSwerve.getPose().transformBy(TurretConstants.kTurretOffset)).getNorm(),
      () -> FieldMathHelpers.Location.ALLIANCE_ZONE));
    NamedCommands.registerCommand("FlywheelOff", new InstantCommand(() -> {
      mFlywheel.setMode(FlywheelMode.OFF, 0.0);
    }));
    NamedCommands.registerCommand("IntakeOn", new setIntakeAction(mIntake, IntakeActionMode.INTAKE));
    NamedCommands.registerCommand("IntakeOff", new setIntakeAction(mIntake, IntakeActionMode.OFF));
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Preload Right Auto", new PathPlannerAuto("Preload Right Auto"));
    autoChooser.addOption("Preload Left Auto", new PathPlannerAuto("Preload Left Auto"));
    autoChooser.addOption("Shoot Preload From Standstill", new ShootPreloadFromStandstill(mFlywheel, mSwerve, mSpindexer));
    autoChooser.addOption("Depot From Center", new PathPlannerAuto("Depo zone"));
    autoChooser.addOption("Right Neutral Zone", new PathPlannerAuto("Neutral zone right side"));
    autoChooser.addOption("Left Neutral Zone", new PathPlannerAuto("Neutral zone agressive"));
    autoChooser.setDefaultOption("Preload Center Auto", new PathPlannerAuto("Preload Auto"));
    SmartDashboard.putData("Auto Selector", autoChooser);

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // Spindexer Bindings
    mDriver.leftBumper().whileTrue(
      new setSpindexer(
        mSpindexer, 
        SpindexerMode.LOAD,
        mTurret::getTurretAtSetpoint,
        () -> FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(mSwerve.getPose(), mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond, mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond, mSwerve.getAngularVelocity()).getNorm(),
        () -> FieldMathHelpers.getLocation(mSwerve.getPose())).unless(() -> !mFlywheel.getFlywheelAtTarget()));

    mDriver.leftBumper().onFalse(
      new setSpindexer(mSpindexer, SpindexerMode.OFF, () -> true, () -> 0.0, () -> FieldMathHelpers.Location.ALLIANCE_ZONE)
    );

    // Flywheel and Hood Bindings
    mDriver.leftTrigger(OperatorConstants.kTriggerThreshold).whileTrue(new ConditionalCommand(new setFlywheelSafe(mFlywheel), new setFlywheel(mFlywheel,
    () -> FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(
        mSwerve.getPose(),
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond,
        mSwerve.getAngularVelocity()).getNorm(),
      () -> mSwerve.getLocation()),
      mFlywheel::getSafeModeEnabled));

    mDriver.leftTrigger(OperatorConstants.kTriggerThreshold).onFalse(new InstantCommand(() -> {
      mFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
      mFlywheel.setMode(FlywheelMode.OFF, 0);
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

    // TODO: make this a constant
    mOperator.y().onTrue(new InstantCommand(() -> mClimb.setVoltage(8)));
    mOperator.a().onTrue(new InstantCommand(() -> mClimb.setVoltage(-8)));
    mOperator.y().or(mOperator.a()).onFalse(new InstantCommand(() -> mClimb.setVoltage(0)));

    mOperator.x().onTrue(new InstantCommand(() -> mSwerve.resetOdometry(mVisionSubsystem.getLastValidPose())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
