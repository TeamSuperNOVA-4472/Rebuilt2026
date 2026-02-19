// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldMathHelpers;
import frc.robot.subsystems.SwerveSubsystem;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
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
  private final SwerveSubsystem mSwerveSubsystem;
  private final Supplier<Boolean> mTurnToHeading; 

  private final PIDController mGyroController = new PIDController(0.1, 0, 0.0005);
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
    Supplier<Boolean> pTurnToHeading,
    SwerveSubsystem pSwerveSubsystem) {
  
    mFwdInput = pFwdInput;
    mSideInput = pSideInput;
    mTurnInput = pTurnInput;
    mResetHeadingInput = pResetHeadingInput;
    mSwerveSubsystem = pSwerveSubsystem;
    mTurnToHeading = pTurnToHeading;

    mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
  
    mGyroController.enableContinuousInput(0, 360);

    addRequirements(pSwerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("Distance to Hub: ", FieldMathHelpers.getDistanceToHub(mSwerveSubsystem.getPose()));

    double updatedFwdSpeedMS = mFwdInput.get() * kMaxSpeedMS;
    double updatedSideSpeedMS = mSideInput.get() * kMaxSpeedMS;
    double updatedTurnSpeedRadS =
      mTurnInput.get() * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;

    if(mTurnToHeading.get())
    {
      double normalizeDegrees = (mSwerveSubsystem.getHeadingDegrees() + 360) % 360;
      // updatedTurnSpeedRadS = mGyroController.calculate(normalizeDegrees, 
      // FieldMathHelpers.getHeadingToHubWithSomeSpeedInDegrees(
      // mSwerveSubsystem.getPose(), mSwerveSubsystem.getFieldRelativeSpeeds().vxMetersPerSecond, mSwerveSubsystem.getFieldRelativeSpeeds().vyMetersPerSecond, mFlywheelSpeed.get()));
      updatedTurnSpeedRadS = mGyroController.calculate(normalizeDegrees, FieldMathHelpers.getHeadingToHubInDegrees(mSwerveSubsystem.getPose()));
    }
    else if(updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0)) {
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
