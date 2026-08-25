// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.configurations.ISwerveModuleConfiguration;
import frc.robot.subsystems.swerve.configurations.SparkMaxDriveTurnAnalogEncConfig;
import frc.robot.util.PIDConstants;
import static frc.robot.Constants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule[] mModules;
  private final SwerveDriveKinematics mKinematics;
  private final AHRS mGyro;
  private final SwerveDriveOdometry mOdometry;

  private double mTargetRobotGyroHeading = 0.0;
  private double mFieldGyroHeading = 0.0;
  private boolean mSkipInitialization = false;

  public SwerveDriveSubsystem() {
    mGyro = new AHRS(NavXComType.kMXP_SPI);
    ISwerveModuleConfiguration frontLeftConfiguration = new SparkMaxDriveTurnAnalogEncConfig.Builder()
      .withDriveCanId(FRONT_LEFT_DRIVE_CAN_ID)
      .withTurnCanId(FRONT_LEFT_TURN_CAN_ID)
      .withTurnAbsEncAnalogPortAndOffset(FRONT_LEFT_TURN_ABS_ENC_PORT, FRONT_LEFT_TURN_ABS_ENC_OFFSET)
      .withTurnPIDConstants(new PIDConstants(TURN_P, TURN_I, TURN_D))
      .withDrivePIDConstants(new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F))
      .build();
    ISwerveModuleConfiguration frontRightConfiguration = new SparkMaxDriveTurnAnalogEncConfig.Builder()
      .withDriveCanId(FRONT_RIGHT_DRIVE_CAN_ID)
      .withTurnCanId(FRONT_RIGHT_TURN_CAN_ID)
      .withTurnAbsEncAnalogPortAndOffset(FRONT_RIGHT_TURN_ABS_ENC_PORT, FRONT_RIGHT_TURN_ABS_ENC_OFFSET)
      .withTurnPIDConstants(new PIDConstants(TURN_P, TURN_I, TURN_D))
      .withDrivePIDConstants(new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F))
      .build();
    ISwerveModuleConfiguration rearLeftConfiguration = new SparkMaxDriveTurnAnalogEncConfig.Builder()
      .withDriveCanId(REAR_LEFT_DRIVE_CAN_ID)
      .withTurnCanId(REAR_LEFT_TURN_CAN_ID)
      .withTurnAbsEncAnalogPortAndOffset(REAR_LEFT_TURN_ABS_ENC_PORT, REAR_LEFT_TURN_ABS_ENC_OFFSET)
      .withTurnPIDConstants(new PIDConstants(TURN_P, TURN_I, TURN_D))
      .withDrivePIDConstants(new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F))
      .build();
    ISwerveModuleConfiguration rearRightConfiguration = new SparkMaxDriveTurnAnalogEncConfig.Builder()
      .withDriveCanId(REAR_RIGHT_DRIVE_CAN_ID)
      .withTurnCanId(REAR_RIGHT_TURN_CAN_ID)
      .withTurnAbsEncAnalogPortAndOffset(REAR_RIGHT_TURN_ABS_ENC_PORT, REAR_RIGHT_TURN_ABS_ENC_OFFSET)
      .withTurnPIDConstants(new PIDConstants(TURN_P, TURN_I, TURN_D))
      .withDrivePIDConstants(new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F))
      .build();
    
    mModules = new SwerveModule[] { 
      new SwerveModule("FL", frontLeftConfiguration),
      new SwerveModule("FR", frontRightConfiguration),
      new SwerveModule("RL", rearLeftConfiguration),
      new SwerveModule("RR", rearRightConfiguration)
    };
  
    mKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.SWERVE_LOC_Y_METERS, Constants.SWERVE_LOC_X_METERS),
      new Translation2d(Constants.SWERVE_LOC_Y_METERS, -Constants.SWERVE_LOC_X_METERS),
      new Translation2d(-Constants.SWERVE_LOC_Y_METERS, Constants.SWERVE_LOC_X_METERS),
      new Translation2d(-Constants.SWERVE_LOC_Y_METERS, -Constants.SWERVE_LOC_X_METERS));
    mOdometry = new SwerveDriveOdometry(mKinematics, Rotation2d.fromDegrees(getRobotGyroHeading()), getModulePositions());
    reset();
  }

  @Override
  public void periodic() {
    Pose2d currentPose = mOdometry.update(Rotation2d.fromDegrees(getRobotGyroHeading()), getModulePositions());
    for(SwerveModule module : mModules) {
      module.outputDebug();
    }

    SmartDashboard.putNumber("Odo X:", currentPose.getX());
    SmartDashboard.putNumber("Odo Y:", currentPose.getY());
    SmartDashboard.putNumber("Odo Heading:", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Gyro (Deg)", getRobotGyroHeading());
    SmartDashboard.putNumber("Target Gyro (Deg)", getTargetRobotGyroHeading());
    SmartDashboard.putNumber("Field Gyro (Deg)", getFieldGyroHeading());
    //SmartDashboard.putNumber("Robot Field (Deg)", getRobotFieldHeading());
    SmartDashboard.putNumber("Target Field (Deg)", getTargetRobotFieldHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void driveFieldOriented(ChassisSpeeds speeds, boolean pIsOpenLoop) {
    driveFieldOriented(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, pIsOpenLoop);
  }

  public void driveRobotOriented(ChassisSpeeds speeds, boolean pIsOpenLoop) {
    driveFieldOriented(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, pIsOpenLoop);
  }

  public void driveRobotOriented(double pMetersPerSecondFwd, double pMetersPerSecondSide, double pRadiansPerSecondTurn, boolean pIsOpenLoop) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(pMetersPerSecondFwd, pMetersPerSecondSide, pRadiansPerSecondTurn);
    SwerveModuleState[] states = mKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states, pIsOpenLoop);
  }

  public void setModuleStates(SwerveModuleState[] pStates, boolean pIsOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(pStates, Constants.MAXIMUM_SPEED_METERS_PER_SECONDS);                   
    for(int i = 0; i < mModules.length; i++) {
      mModules[i].setState(pStates[i], pIsOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] pStates) {
    setModuleStates(pStates, false);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[mModules.length];
    for(int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getPosition();
    }
    return states;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[mModules.length];
    for(int i = 0; i < mModules.length; i++) {
      states[i] = mModules[i].getState();
    }
    return states;
  }

  public void driveFieldOriented(double pMetersPerSecondFwd, double pMetersPerSecondSide, double pRadiansPerSecondTurn, boolean pIsOpenLoop) {
    ChassisSpeeds translatedSpeeds = 
        ChassisSpeeds.fromFieldRelativeSpeeds(pMetersPerSecondFwd, pMetersPerSecondSide, pRadiansPerSecondTurn, mOdometry.getPoseMeters().getRotation());
    driveRobotOriented(translatedSpeeds.vxMetersPerSecond, translatedSpeeds.vyMetersPerSecond, translatedSpeeds.omegaRadiansPerSecond, pIsOpenLoop);
  }

  public void reset() {
    for(SwerveModule module : mModules) {
      module.reset();
    }
    resetFieldGryoHeading();
    resetTargetRobotGryoHeading();
  }

  public SwerveDriveKinematics getSwerveKinematics() {
    return mKinematics;
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pPose) {
    mOdometry.resetPosition(Rotation2d.fromDegrees(getRobotGyroHeading()), getModulePositions(), pPose);
  }

  /**
   * Gets the heading robot heading relative to the initial gyroscope value.
   * @return The desired heading in degrees from 0 to 360.
   */
  public double getRobotGyroHeading() {
    double gyroAngle = -mGyro.getYaw() * GYRO_ERROR_FACTOR;
    return (gyroAngle % 360 + 360) % 360;
  }

  /**
   * Gets the field heading relative to the initial gyroscope value.
   * @return The desired heading in degrees.
   */
  public double getFieldGyroHeading () {
    return mFieldGyroHeading;
  }

  /**
   * Gets the target robot heading relative to the initial gyroscope value
   * @return The desired heading in degrees.
   */
  public double getTargetRobotGyroHeading() {
    return mTargetRobotGyroHeading;
  }

  // public double getRobotFieldHeading() {
  //   double gyroAngle = -mGyro.getAngle() * GYRO_ERROR_FACTOR;
  //   double robotFieldHeading = gyroAngle - mFieldGyroHeading;
  //   return (robotFieldHeading % 360 + 360) % 360;
  // }

  public double getTargetRobotFieldHeading() {
    double targetRobotFieldHeading = mTargetRobotGyroHeading - mFieldGyroHeading;
    return (targetRobotFieldHeading % 360 + 360) % 360;
  }

  public void resetTargetRobotGryoHeading() {
    mTargetRobotGyroHeading = getRobotGyroHeading();
  }

  public void resetFieldGryoHeading() {
    mFieldGyroHeading = getRobotGyroHeading();
    Pose2d oldPose = getPose();
    Pose2d newPose = new Pose2d(oldPose.getX(), oldPose.getY(), Rotation2d.fromDegrees(0));
    resetOdometry(newPose);
  }

  /**
   * Sets the field heading based off an offset relative to the robot.
   * @param pFieldHeadingOffset The field heading offset in degrees.
   */
  public void setFieldHeading(double pFieldHeadingOffset) {
    mFieldGyroHeading =  ((getRobotGyroHeading() + pFieldHeadingOffset) % 360 + 360) % 360;
  }

  public AHRS getGyro(){
    return mGyro;
  }

  public void setSkipInitialization(boolean pSkipInitialization) {
    mSkipInitialization = pSkipInitialization;
  }

  public boolean skipInitialization() {
    return mSkipInitialization;
  }
}
