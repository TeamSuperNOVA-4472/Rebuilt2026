// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import frc.robot.Constants;
import frc.robot.FieldMathHelpers;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.SwerveConstants.*;

/** An example command that uses an example subsystem. */
public class SwerveTeleop extends Command {

  private final Supplier<Double> mFwdInput;
  private final Supplier<Double> mSideInput;
  private final Supplier<Double> mTurnInput;
  private final Supplier<Boolean> mResetHeadingInput;
  private final Supplier<Double> mSOTM;
  private final SwerveSubsystem mSwerveSubsystem;
  
  private final PIDController mGyroController = new PIDController(Constants.SwerveConstants.kPGyro, Constants.SwerveConstants.kIGyro, Constants.SwerveConstants.kDGyro);
  private double mTargetHeading;

  /**
   * Creates a new ExampleCommand.
   *
   * @param pSwerveSubsystem The subsystem used by this command.
   */
  public SwerveTeleop(Supplier<Double> pFwdInput,
    Supplier<Double> pSideInput,
    Supplier<Double> pTurnInput,
    Supplier<Boolean> pResetHeadingInput,
    Supplier<Double> pSOTM,
    SwerveSubsystem pSwerveSubsystem) {
  
    mFwdInput = pFwdInput;
    mSideInput = pSideInput;
    mTurnInput = pTurnInput;
    mResetHeadingInput = pResetHeadingInput;
    mSOTM = pSOTM;
    mSwerveSubsystem = pSwerveSubsystem;

    mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
  
    mGyroController.enableContinuousInput(0, 360);

    addRequirements(pSwerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //TODO: make analog transition w/ trigger so it isnt as choppy
    double speed = MathUtil.clamp(mSOTM.get() * SwerveConstants.kSOTMConstant + SwerveConstants.kMaxSpeedMS, SwerveConstants.kMaxSOTMSpeedMS, SwerveConstants.kMaxSpeedMS);

    double updatedFwdSpeedMS = mFwdInput.get() * speed;
    double updatedSideSpeedMS = mSideInput.get() * speed;
    double updatedTurnSpeedRadS = mTurnInput.get() * kMetersPerSecondToRadiansPerSecond * speed;

    if(updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0)) {
      updatedTurnSpeedRadS = mGyroController.calculate(mSwerveSubsystem.getHeadingDegrees(), mTargetHeading);
    } else {
      mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
    }

    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
      updatedFwdSpeedMS,
      updatedSideSpeedMS, 
      updatedTurnSpeedRadS);
    mSwerveSubsystem.driveFieldOriented(updatedSpeeds);

    if(mResetHeadingInput.get()) {
      mSwerveSubsystem.resetHeading();
    }

  }

}
