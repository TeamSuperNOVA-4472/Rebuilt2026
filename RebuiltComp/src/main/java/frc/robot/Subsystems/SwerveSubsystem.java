// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.FieldMathHelpers;
import frc.robot.FieldMathHelpers.Location;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.Constants.SwerveConstants.*;


public class SwerveSubsystem extends SubsystemBase {
  private final Field2d mField = new Field2d();
  private final SwerveDrive mSwerveDrive;
  private double kYawGyroOffset = 0;
  private Location kLocation = FieldMathHelpers.Location.ALLIANCE_ZONE;

  private static SwerveDrive readSwerveConfig() {
    SwerveDrive swerveDrive = null;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
        .createSwerveDrive(kMaxSpeedMS);
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(SwerveConstants.kSReplacement, SwerveConstants.kVReplacement, SwerveConstants.kAReplacement));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    return swerveDrive;
  }

  private static void configAutoBuilder(SwerveSubsystem pSwerveSubsystem) {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
        pSwerveSubsystem::getPose, // Robot pose supplier
        (Pose2d pose) -> pSwerveSubsystem.resetOdometry(pose), // Method to reset odometry (will be called if your auto has a starting pose)
        pSwerveSubsystem::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> pSwerveSubsystem.driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(SwerveConstants.kPTranslation, 0.0, 0.0), // Translation PID constants
                new PIDConstants(SwerveConstants.kPRotation, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        pSwerveSubsystem // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  // Swerve subsystem constructor 
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
    mSwerveDrive = readSwerveConfig();
    mSwerveDrive.setHeadingCorrection(false);
    configAutoBuilder(this);
    resetHeading();
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  // Drive in some direction with its reference point set to the field itself.
  public void driveFieldOriented(ChassisSpeeds pVelocity)
  {
      if(isRedAlliance()) {
        ChassisSpeeds fieldOrientedVelocity =
          ChassisSpeeds.fromFieldRelativeSpeeds(
            pVelocity,
            mSwerveDrive.getOdometryHeading().plus(Rotation2d.fromRadians(Math.PI)));
        mSwerveDrive.drive(fieldOrientedVelocity);
      }
      else {
        mSwerveDrive.driveFieldOriented(pVelocity);
      }
  }
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs)
  {
    mSwerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  public void driveRobotOriented(ChassisSpeeds pVelocity) {
    mSwerveDrive.drive(pVelocity);
    
  }

  public void resetOdometry(Pose2d pPose) {
    mSwerveDrive.resetOdometry(pPose);
    kYawGyroOffset = ((pPose.getRotation().getDegrees() - mSwerveDrive.getYaw().getDegrees()) % 360 + 360) % 360;
  }

  public void resetHeading() {
    Rotation2d newHeading = Rotation2d.fromRadians(0);
    if(isRedAlliance()) {
      newHeading = Rotation2d.fromRadians(Math.PI);
    }
    resetOdometry(new Pose2d(getPose().getTranslation(), newHeading));
  }

  public Pose2d getPose() {
    return mSwerveDrive.getPose();
  }

  public Pose2d getFuturePose() {
    ChassisSpeeds speeds = getFieldRelativeSpeeds();
    double dt = SwerveConstants.kLatencyInSeconds;

    Pose2d currentPose = getPose();
    double futureX = currentPose.getX() + dt * speeds.vxMetersPerSecond;
    double futureY = currentPose.getY() + dt * speeds.vyMetersPerSecond;

    return new Pose2d(futureX, futureY, currentPose.getRotation());
  }

  public Location getLocation() {
    return kLocation;
  }

  public double getHeadingDegrees() {
    return (((mSwerveDrive.getYaw().getDegrees() + kYawGyroOffset) % 360) + 360) % 360;
  }

  public double getAngularVelocity() {
    return Units.radiansToDegrees(getRobotRelativeSpeeds().omegaRadiansPerSecond);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return mSwerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return mSwerveDrive.getFieldVelocity();
  }

  @Override
  public void periodic() {
    kLocation = FieldMathHelpers.getLocation(getPose());
    mField.setRobotPose(getPose());
  }
}
