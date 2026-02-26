// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.SwerveTeleop;
import frc.robot.Commands.setFlywheel;
import frc.robot.Commands.MoveTurretAbsolute;
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
  
    
  private final setFlywheel mSetFlywheel = new setFlywheel(
    mFlywheel,
    () -> 20.0, 
    () -> mDriver.leftTrigger().getAsBoolean());

  
  private final MoveTurretAbsolute mMoveTurretAbsolute = new MoveTurretAbsolute(
      mTurret, 
      mSwerve::getHeadingDegrees, 
      () -> FieldMathHelpers.getRotationToHubWithSomeSpeed(
        mSwerve.getPose(), 
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond));

  public RobotContainer() {
    //mSwerve.setDefaultCommand(mSwerveTeleop);
    //mTurret.setDefaultCommand(mMoveTurretAbsolute);
    //mFlywheel.setDefaultCommand(mSetFlywheel);

    mVisionSubsystem = new VisionSubsystem(mSwerve::getHeadingDegrees, mSwerve::getAngularVelocity,
    (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
      mSwerve.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
    });

    configureBindings();
  }

  private void configureBindings() {
    //mDriver.rightBumper().whileTrue(new setIntakeAction(mIntake, IntakeActionMode.INTAKE));
    //mDriver.rightTrigger().whileTrue(new setIntakeAction(mIntake, IntakeActionMode.OUTTAKE));

    //mDriver.leftBumper().whileTrue(new setSpindexer(mSpindexer, SpindexerMode.LOAD));

    //mDriver.back().onTrue(new toggleIntakeStorage(mIntake));

    //TODO: add controls for flywheel + add correct bindings
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
