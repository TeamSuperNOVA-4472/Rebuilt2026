// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.SwerveTeleop;
import frc.robot.Commands.flywheelSysIDCommand;
import frc.robot.Commands.setFlywheel;
import frc.robot.Commands.setFlywheelTest;
import frc.robot.Commands.moveTurretAbsolute;
import frc.robot.Commands.setIntakeAction;
import frc.robot.Commands.setSpindexer;
import frc.robot.Commands.toggleIntakeStorage;
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

public class RobotContainer {
  private double mAngle = 30;
  private final IntakeSubsystem mIntake = IntakeSubsystem.kIntake;
  private final SpindexerSubsystem mSpindexer = SpindexerSubsystem.kSpindexer;
  private final VisionSubsystem mVisionSubsystem;
  private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final TurretSubsystem mTurret = TurretSubsystem.kTurret;
  private final SwerveSubsystem mSwerve = SwerveSubsystem.kSwerve;
  private final FlywheelSubsystem mFlywheel = FlywheelSubsystem.kFlywheel;
  
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
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond));

  public RobotContainer() {
    mSwerve.setDefaultCommand(mSwerveTeleop);
    //mTurret.setDefaultCommand(mMoveTurretAbsolute);

    mVisionSubsystem = new VisionSubsystem(mSwerve::getHeadingDegrees, mSwerve::getAngularVelocity,
    (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
      mSwerve.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
    });

    configureBindings();
  }

  private void configureBindings() {

    mDriver.leftBumper().onTrue(new setSpindexer(mSpindexer, SpindexerMode.LOAD));
    mDriver.leftBumper().onFalse(new setSpindexer(mSpindexer, SpindexerMode.OFF));

    mDriver.rightBumper().onTrue(new setIntakeAction(mIntake, IntakeActionMode.INTAKE));
    mDriver.rightTrigger(0.2).onTrue(new setIntakeAction(mIntake, IntakeActionMode.OUTTAKE));

    mDriver.rightBumper().or(mDriver.rightTrigger(0.2)).onFalse(new setIntakeAction(mIntake, IntakeActionMode.OFF));
    /*mDriver.leftTrigger(0.2).whileTrue(new setFlywheel(mFlywheel, () -> FieldMathHelpers.getDistanceToHubWithSomeSpeed(
        mSwerve.getPose(), 
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond)));*/
        
    mDriver.leftTrigger(0.2).whileTrue(new setFlywheelTest(
        mFlywheel, 
        mDriver.povUp()::getAsBoolean, 
        mDriver.povDown()::getAsBoolean,
        mDriver.povLeft()::getAsBoolean,
        mDriver.povRight()::getAsBoolean));

    mDriver.leftTrigger(0.2).onFalse(new InstantCommand(() -> {
      mFlywheel.setMode(FlywheelMode.OFF, 0);
    }));

    //mDriver.y().onTrue(new InstantCommand(() -> mIntake.moveIntake(0.1)));
    //mDriver.x().onTrue(new InstantCommand(() -> mIntake.moveIntake(-0.1)));
    //mDriver.y().or(mDriver.x()).onFalse(new InstantCommand(() -> mIntake.stopIntake()));
    mDriver.back().onTrue(new toggleIntakeStorage(mIntake));

    //TODO: add controls for flywheel + add correct bindings
  }

  public Command getAutonomousCommand() {
    return new flywheelSysIDCommand(mFlywheel);
  }
}
