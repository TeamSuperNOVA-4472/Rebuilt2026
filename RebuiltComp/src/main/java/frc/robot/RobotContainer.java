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
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Commands.SwerveTeleop;
import frc.robot.Commands.moveTurretAbsolute;
import frc.robot.Commands.setFlywheelHoodAngle;
import frc.robot.Commands.setIntake;
import frc.robot.Commands.setSpindexer;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;
import frc.robot.Subsystems.IntakeSubsystem.IntakeMode;
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
  private final moveTurretAbsolute mMoveTurretAbsolute = new moveTurretAbsolute(
      mTurret, 
      mSwerve::getHeadingDegrees, 
      () -> FieldMathHelpers.getRotationToHubWithSomeSpeed(
        mSwerve.getPose(), 
        mSwerve.getFieldRelativeSpeeds().vxMetersPerSecond,
        mSwerve.getFieldRelativeSpeeds().vyMetersPerSecond));

  public RobotContainer() {
    mSwerve.setDefaultCommand(mSwerveTeleop);
    if (Robot.isReal())mTurret.setDefaultCommand(mMoveTurretAbsolute);

    mVisionSubsystem = new VisionSubsystem(mSwerve::getHeadingDegrees, mSwerve::getAngularVelocity,
    (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
      mSwerve.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
    });

    configureBindings();
  }

  private void configureBindings() {
    mDriver.leftBumper().onTrue(new setIntake(mIntake, IntakeMode.INTAKE));
    mDriver.rightBumper().onTrue(new setIntake(mIntake, IntakeMode.OUTTAKE));
    mDriver.a().onTrue(new setSpindexer(mSpindexer, SpindexerMode.LOAD));
    mDriver.a().onFalse(new setSpindexer(mSpindexer, SpindexerMode.OFF));
    mDriver.b().onTrue(new setFlywheelHoodAngle(mFlywheel,35));
    //TODO: add controls for flywheel + add correct bindings
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
