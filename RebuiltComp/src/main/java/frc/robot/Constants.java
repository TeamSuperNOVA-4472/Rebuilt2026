// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstants {
    public static final double kMaxSpeedMS = 4.5;
    public static final double kMetersPerInch = Units.inchesToMeters(1);
    public static final double kSwerveLocYInches = 7.5;
    public static final double kSwerveLocXInches = 7;
    public static final double kSwerveLocYMeters = kSwerveLocYInches * kMetersPerInch;
    public static final double kSwerveLocXMeters = kSwerveLocXInches * kMetersPerInch;
    public static final double kSwerveRadiusInches = Math.sqrt(Math.pow(kSwerveLocXInches, 2) + Math.pow(kSwerveLocYInches, 2));
    public static final double kSwerveCircumferenceMeters = 2 * Math.PI * kSwerveRadiusInches * kMetersPerInch; //1.6372863352652361048052029816421;
    public static final double kMetersPerSecondToRadiansPerSecond = (2 * Math.PI) / kSwerveCircumferenceMeters;
    public static final double kS = 0.267;
    public static final double kV = 2.65;
    public static final double kA = 0.239;
    public static final double kPGyro = 0.12;
    public static final double kIGyro = 0;
    public static final double kDGyro = 0.0005;
  }

  public static class ClimbConstants {
    public static final int kClimbMotorPort = 59;
    public static final String kClimbCanbus = "CANivore";
    public static final double kClimbSupplyLimit = 40;
    public static final double kClimbStatorLimit = 40;
    public static final boolean kClimbSupplyLimitEnabled = true;
    public static final boolean kClimbStatorLimitEnabled = true;
    public static final NeutralModeValue kClimbNeutralMode = NeutralModeValue.Brake;

    public static final double kGearing = 1/45;
  }

  public static class FlywheelConstants {
    public static final InterpolatingDoubleTreeMap kDistanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
    static {
      kDistanceToFlywheelSpeed.put(1.8161, 40.0);
      kDistanceToFlywheelSpeed.put(2.5781, 45.0);
      kDistanceToFlywheelSpeed.put(3.3401, 50.0);
      kDistanceToFlywheelSpeed.put(4.1021, 60.0);
      kDistanceToFlywheelSpeed.put(4.8641, 72.0);
    }

    public static final InterpolatingDoubleTreeMap kDistanceToFlywheelSpeedTime = new InterpolatingDoubleTreeMap();
    static {
      kDistanceToFlywheelSpeedTime.put(1.8161, 1.04);
      kDistanceToFlywheelSpeedTime.put(2.5781, 1.07);
      kDistanceToFlywheelSpeedTime.put(3.3401,1.05);
      kDistanceToFlywheelSpeedTime.put(4.1021, 1.08);
      kDistanceToFlywheelSpeedTime.put(4.8641, 1.24);
    }

    public static final InterpolatingDoubleTreeMap kDistanceToHoodAngle = new InterpolatingDoubleTreeMap();
    static {
      kDistanceToHoodAngle.put(1.8161, 20.0);
      kDistanceToHoodAngle.put(2.5781, 24.0);
      kDistanceToHoodAngle.put(3.3401,26.0);
      kDistanceToHoodAngle.put(4.1021, 28.0);
      kDistanceToHoodAngle.put(4.8641, 28.0);
    }

    public static final double kHoodEncoderMultiplier = (0.02833333333330) * 360.0;
    public static final double kHoodMinAngle = 20.0;
    public static final double kHoodMaxAngle = 45.0;
    public static final double kDistanceThresholdInMeters = 1.8161;
    public static final double kDistanceMaximumInMeters = 4.8641;

    public static final int kFlywheel1MotorPort = 60;
    public static final String kFlywheel1Canbus = "rio";
    public static final double kFlywheel1SupplyLimit = 20;
    public static final double kFlywheel1StatorLimit = 20;
    public static final boolean kFlywheel1SupplyLimitEnabled = true;
    public static final boolean kFlywheel1StatorLimitEnabled = true;
    public static final NeutralModeValue kFlywheel1NeutralMode = NeutralModeValue.Coast;

    public static final int kFlywheel2MotorPort = 20;
    public static final String kFlywheel2Canbus = "rio";
    public static final double kFlywheel2SupplyLimit = 20;
    public static final double kFlywheel2StatorLimit = 20;
    public static final boolean kFlywheel2SupplyLimitEnabled = true;
    public static final boolean kFlywheel2StatorLimitEnabled = true;
    public static final NeutralModeValue kFlywheel2NeutralMode = NeutralModeValue.Coast;

    public static final int kFlywheelHoodMotorPort = 41;
    public static final String kFlywheelHoodCanbus = "rio";
    public static final double kFlywheelHoodSupplyLimit = 20;
    public static final double kFlywheelHoodStatorLimit = 20;
    public static final boolean kFlywheelHoodSupplyLimitEnabled = true;
    public static final boolean kFlywheelHoodStatorLimitEnabled = true;
    public static final NeutralModeValue kFlywheelHoodNeutralMode = NeutralModeValue.Brake;

    public static final double kPHood = 0.017;
    public static final double kIHood = 0;
    public static final double kDHood = 0;

    public static final double kHoodTolerance = 2;

    public static final double kSFlywheel = 0.44;
    public static final double kVFlywheel = 0.12;
    public static final double kAFlywheel = 0;

    public static final double kPFlywheel = 0.015;
    public static final double kIFlywheel = 0;
    public static final double kDFlywheel = 0;

    public static final double kFlywheelTolerance = 0.2;
    
    public static final double kStartingHoodAngle = 21;
    public static final double kPassingAngle = 44;
    public static final double kPassingSpeed = 70;
    public static final double kSafeAngle = 21;
    public static final double kSafeSpeed = 40;

    public static final double kResetHoodSpeed = -0.1;
    public static final double kResetHoodStatorThreshold = 18;

    public static final int kSimNumMotors = 1;
    public static final double kSimGearing = 35.294;
    public static final double kSimjKgMetersSquared = 0.011;
    public static final double kSimArmLength = 0.2159;
    public static final double kSimWidth = 60;
    public static final double kSimHeight = 60;
    public static final String kSimRootName = "base";
    public static final double kSimX = 30;
    public static final double kSimY = 30;
    public static final String kSimName = "Turret";
    public static final double kSimLength = 10;
    public static final double kSimdt = 0.02;
    public static final double kSimMultiplier = 12;

    public static final double kMaxSpeed = 1;
    public static final double kMaxVoltage = 11.5;
  }

  public static class OperatorConstants {
    public static final double kDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kTriggerThreshold = 0.2;
    public static final InterpolatingDoubleTreeMap kControllerProfileMap = new InterpolatingDoubleTreeMap();
    public static final double kSlewLimit = 1.0;

    static {
        kControllerProfileMap.put(0.0, 0.0);
        // kControllerProfileMap.put(0.8, 1.0/3.0);
        kControllerProfileMap.put(1.0, 1.0);
    }

    public static double getControllerProfileValue(double pValue) {
      double deadbandedVal = MathUtil.applyDeadband(pValue, kDeadband);
      double clampedVal = MathUtil.clamp(deadbandedVal, -1, 1);
      return Math.signum(clampedVal) * kControllerProfileMap.get(Math.abs(clampedVal));
    }
  }

  public static class VisionConstants {
    public static final boolean kUseMegatag2ByDefault = true; 
    public static final String[] kLimelightNames = {"limelight-one","limelight-two"};
    public static final Matrix<N3, N1> kStandardDeviations = VecBuilder.fill(.7,.7,9999999);
    public static final double kAmbiguity = .9;
    public static final double kBaseLateralDev = 0.3;
    public static final double kBaseRotDev = 0.5;
    public static final double kTagDistThreshold = 10;
    public static final double kLatencyLagInSeconds = 0.4;
    public static final double kTagCountThreshold = 1;
    public static final int kThrottle = 200;
    public static final double kAngularVelocityThreshold = 720;
    public static final double kCooldownBump = 0.5;

    public static final boolean kIsAndyMark = false;

    // Welded hub poses
    public static final Pose2d kHubPoseBlueWeldedMeters = new Pose2d(4.6255177999999995, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kHubPoseRedWeldedMeters = new Pose2d(11.9155209999999985, 4.0346376, Rotation2d.fromDegrees(0));

    // Andymark hub poses
    public static final Pose2d kHubPoseBlueAndyMarkMeters = new Pose2d(4.6115224, 4.0213534, Rotation2d.fromDegrees(0));
    public static final Pose2d kHubPoseRedAndyMarkMeters = new Pose2d(11.9015002, 4.0213534, Rotation2d.fromDegrees(0));
  
    // AdvantageKit mode constants
    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    String logPath = LogFileUtil.findReplayLog();

    public static enum Mode {
      // Running on a real robot.
      REAL,

      //Running a physics simulator.
      SIM,

      //Replaying from a log file.
      REPLAY
    }

    public static final double kRedTrenchXHighThreshold = 11.938;
    public static final double kRedTrenchXLowThreshold = 11.8618;

    public static final double kBlueTrenchXHighThreshold = 4.648962;
    public static final double kBlueTrenchXLowThreshold = 4.574032;

    public static final double kTopTrenchYThreshold = 7.4350372;
    public static final double kBottomTrenchYThreshold = 1.268476;

    public static final double kRedBumpXHighThreshold = 12.498324;
    public static final double kRedBumpXLowThreshold = 11.304524;

    public static final double kBlueBumpXHighThreshold = 5.208524;
    public static final double kBlueBumpXLowThreshold = 4.014724;
  }

  
  public static class TurretConstants {
    public static final Transform2d kTurretOffset = new Transform2d(0.146,-0.171, new Rotation2d());
    public static final double kStartingAngleOffset = 87;

    public static final int kTurretMotorPort = 24;
    public static final String kTurretCanbus = "CANivore";
    public static final double kTurretSupplyLimit = 20;
    public static final double kTurretStatorLimit = 20;
    public static final boolean kTurretSupplyLimitEnabled = true;
    public static final boolean kTurretStatorLimitEnabled = true;
    public static final NeutralModeValue kTurretNeutralMode = NeutralModeValue.Coast;

    public static final double kTurretP = 0.003;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.0;

    public static final double kTurretS = 0.0125;
    public static final double kTurretV = 0.00055;
    public static final double kTurretA = 0;

    public static final double kTurretResetSpeed = -0.3;
    public static final double kTurretResetStatorThreshold = 19;

    public static final double kDeadband = 330;
    public static final double kGearing = 24.668;

    public static final double kMaxSpeedOutput = 1;

    public static final int kSimNumMotor = 1;
    public static final double kSimjKgMetersSquared = 0.356; 
    public static final double kSimArmLength = 0.2921;
    public static final double kSimWidth = 60;
    public static final double kSimHeight = 60;
    public static final String kSimRootName = "base";
    public static final double kSimX = 30;
    public static final double kSimY = 30;
    public static final double kSimLength = 10;
    public static final double kSimMultiplier = 12;
    public static final double kSimdt = 0.02;
  }

  public static class SpindexerConstants {
    public static final int kSpindexerMotorPort = 30;
    public static final String kSpindexerCanbus = "rio";
    public static final double kSpindexerSupplyLimit = 40;
    public static final double kSpindexerStatorLimit = 40;
    public static final boolean kSpindexerSupplyLimitEnabled = true;
    public static final boolean kSpindexerStatorLimitEnabled = true;
    public static final NeutralModeValue kSpindexerNeutralMode = NeutralModeValue.Brake;

    public static final int kKickerMotorPort = 51;
    public static final String kKickerCanbus = "CANivore";
    public static final double kKickerSupplyLimit = 40;
    public static final double kKickerStatorLimit = 40;
    public static final boolean kKickerSupplyLimitEnabled = true;
    public static final boolean kKickerStatorLimitEnabled = true;
    public static final NeutralModeValue kKickerNeutralMode = NeutralModeValue.Coast;

    public static final double kKickerP = 0.03;
    public static final double kKickerI = 0;
    public static final double kKickerD = 0;

    public static final double kKickerGearing = 0.25;
    public static final double kSpindexerGearing = 0.25;
    public static final double kKickerV = 0.4;

    public static final double kSpindexerVoltage = -3.5;
    public static final double kKickerSpeed = -20;
  }
  
  public static class IntakeSubsystemConstants {
    //TODO: Find real values for the constants.
    public static final double kSliderOffset = 0.0;
    public static final double kStoredPos = 0.0;
    public static final double kOutPos = 11;
    public static final double kIntakeAngle = 345.0;
    public static final double kIntakeMotorSpeed = 0.8;
    public static final double kSlideThreshold = 0.20;
    public static final double kSlideSpeedThreshold = 0.20;

    public static final int kIntakeMotorPort = 12;
    public static final String kIntakeCanbus = "rio";
    public static final double kIntakeSupplyLimit = 40;
    public static final double kIntakeStatorLimit = 40;
    public static final boolean kIntakeSupplyLimitEnabled = true;
    public static final boolean kIntakeStatorLimitEnabled = true;
    public static final NeutralModeValue kIntakeNeutralMode = NeutralModeValue.Coast;

    public static final int kSliderMotorPort = 31;
    public static final String kSliderCanbus = "rio";
    public static final double kSliderSupplyLimit = 30;
    public static final double kSliderStatorLimit = 30;
    public static final boolean kSliderSupplyLimitEnabled = true;
    public static final boolean kSliderStatorLimitEnabled = true;
    public static final NeutralModeValue kSliderNeutralMode = NeutralModeValue.Coast;

    public static final double kSliderP = 0.07;
    public static final double kSliderI = 0;
    public static final double kSliderD = 0.0;

    public static final double kSliderMaxVelocity = 48;
    public static final double kSliderMaxAcceleration = 36;

    public static final double kSliderMaxSpeedOutput = .5;

    public static final double kSliderResetSpeed = .2;
    public static final double kSliderResetStatorThreshold = 28;

    public static final int kSimNumMotors = 1;
    public static final double kSimWidth = 60;
    public static final double kSimHeight = 60;
    public static final double kGearing = 5;
    public static final double kMass = 2.26796;
    public static final double kDrumRadius = 0.01524;
    public static final double kMaxLen = 0.4572;
    public static final double kSimLenMult = 39.3701*3;
    public static final double kSimLenBaseMult = 39.3701;
    public static final double kSimMaxSpeed = 1;
    public static final String kSimRootName = "base";
    public static final double kSimX = 10;
    public static final double kSimY = 30;
    public static final double kSimMultiplier = 12;
    public static final double kSimdt = 0.02;

    public static final double kEncoderToInchesMult = 3.1875 / 5 ;
  }
}
