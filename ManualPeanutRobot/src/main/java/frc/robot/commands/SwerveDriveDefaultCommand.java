// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SwerveDriveDefaultCommand extends Command {

  private final SwerveDriveSubsystem mSubsystem;

  private final XboxController mController;

  private final PIDController mGyroController;

  private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);

  private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);

  private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1.0);

  private boolean mIsFieldOriented;

  private boolean mIsOpenLoop;

  private boolean mApplyGyroCorrection;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDriveDefaultCommand(SwerveDriveSubsystem pSubsystem, XboxController pController) {
    mSubsystem = pSubsystem;
    mController = pController;
    mGyroController = new PIDController(0.10, 0, 0.001);
    mGyroController.enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIsFieldOriented = true;
    mIsOpenLoop = false;
    mApplyGyroCorrection = false;
    if(!mSubsystem.skipInitialization()) {
      //System.out.println("Initializing Default Swerve Command");
      mSubsystem.reset();
      mSubsystem.setSkipInitialization(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read joystick values
    double joyLeftY = -MathUtil.applyDeadband(mController.getLeftY(), 0.1);
    double joyLeftX = -MathUtil.applyDeadband(mController.getLeftX(), 0.1);
    double joyRightX = -MathUtil.applyDeadband(mController.getRightX(), 0.1);
  
    // Apply controller profile curve for better low speed controllability 
    double fwdJoyVal = Math.signum(joyLeftY) * Constants.CONTROLLER_PROFLE_MAP.get(Math.abs(joyLeftY));
    double sideJoyVal = Math.signum(joyLeftX) * Constants.CONTROLLER_PROFLE_MAP.get(Math.abs(joyLeftX));
    double turnJoyVal = Math.signum(joyRightX) * Constants.CONTROLLER_PROFLE_MAP.get(Math.abs(joyRightX));
  
    // Apply slew rate limiter to prevent sudden acceleration that can cause robot tipping
    fwdJoyVal = mFwdLimiter.calculate(fwdJoyVal);
    sideJoyVal = mSideLimiter.calculate(sideJoyVal);
    turnJoyVal = mTurnLimiter.calculate(turnJoyVal);

    // Convert joystick inputs into translational velocities in m/s and turn velocities in rad/s
    double metersPerSecondFwd = Constants.MAXIMUM_SPEED_METERS_PER_SECONDS * fwdJoyVal;
    double metersPerSecondSide = Constants.MAXIMUM_SPEED_METERS_PER_SECONDS * sideJoyVal;
    double radianPerSecondTurn = Constants.MAXIMUM_SPEED_METERS_PER_SECONDS * Constants.METERS_PER_SECONDS_TO_RADIANS_PER_SECONDS * turnJoyVal;
    
    // Check for toggle for robot vs field oriented
    if(mController.getAButtonPressed()) {
      mIsFieldOriented = !mIsFieldOriented;
      // If field oriented is active reset heading and gyro
      if(mIsFieldOriented) {
        mSubsystem.resetFieldGryoHeading();
        mSubsystem.resetTargetRobotGryoHeading();
      }
    }

    // Check toggle for gyro correction
    if(mController.getBButtonPressed()) {
      mApplyGyroCorrection = !mApplyGyroCorrection;
      // If gyro correction is applied obtain the latest heading
      if(mApplyGyroCorrection) {
        mSubsystem.resetTargetRobotGryoHeading();
      }
    }

    if(mApplyGyroCorrection) {
      if(turnJoyVal == 0.0 && (fwdJoyVal != 0 || sideJoyVal != 0)) {
        radianPerSecondTurn = mGyroController.calculate(mSubsystem.getRobotGyroHeading(), mSubsystem.getTargetRobotGyroHeading());
      } else {
        mSubsystem.resetTargetRobotGryoHeading();
      }
    }

    if(mIsFieldOriented) {
      mSubsystem.driveFieldOriented(metersPerSecondFwd, metersPerSecondSide, radianPerSecondTurn, mIsOpenLoop);
    } else {
      mSubsystem.driveRobotOriented(metersPerSecondFwd, metersPerSecondSide, radianPerSecondTurn, mIsOpenLoop);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(RobotState.isDisabled()) {
      mSubsystem.setSkipInitialization(false);
    }
    mSubsystem.driveRobotOriented(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
