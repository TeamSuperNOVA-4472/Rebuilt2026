// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
  }

  public static class FlywheelConstants {
    public static final InterpolatingDoubleTreeMap kDistanceToVelocity = new InterpolatingDoubleTreeMap();
    static {
        kDistanceToVelocity.put(1.85,50.0);
        kDistanceToVelocity.put(2.0,52.0);
        kDistanceToVelocity.put(2.5,54.0);
        kDistanceToVelocity.put(3.5,62.0);
    }

    public static final InterpolatingDoubleTreeMap kVelocityToTangentialSpeed = new InterpolatingDoubleTreeMap();
    static {
      // TODO: add table values
    }
  }

  public static class OperatorConstants {
    public static final double kDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final InterpolatingDoubleTreeMap kControllerProfileMap = new InterpolatingDoubleTreeMap();

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
    public static final boolean kUseMegatag2 = true; 
    public static final String[] kLimelightNames = {"limelight-two"};
    public static final Matrix<N3, N1> kStandardDeviations = VecBuilder.fill(.7,.7,9999999);
    public static final double kAmbiguity = 10000;
    public static final double kBaseLateralDev = 0;
    public static final double kBaseRotDev = 0;
    public static final double kTagDistThreshold =100000;
    public static final double kTagCountThreshold = 1;
    public static final int kThrottle = 200;

    public static final boolean kIsAndyMark = false;

    // Welded hub poses
    public static final Pose2d kHubPoseBlueWeldedMeters = new Pose2d(4.6255177999999995, 4.0346376, Rotation2d.fromDegrees(0));
    public static final Pose2d kHubPoseRedWeldedMeters = new Pose2d(11.9155209999999985, 4.0346376, Rotation2d.fromDegrees(0));

    // Andymark hub poses
    public static final Pose2d kHubPoseBlueAndyMarkMeters = new Pose2d(4.6115224, 4.0213534, Rotation2d.fromDegrees(0));
    public static final Pose2d kHubPoseRedAndyMarkMeters = new Pose2d(11.9015002, 4.0213534, Rotation2d.fromDegrees(0));
  }
  
  public static class TurretConstants {
    public static final Translation2d kTurretOffset = new Translation2d(-0.2,0);
  }

  public static class SpindexerConstants {
    public static final double kSpindexerSpeed = 0.8;
  }
}
