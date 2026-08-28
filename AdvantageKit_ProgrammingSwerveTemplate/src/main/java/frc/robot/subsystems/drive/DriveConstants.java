// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double fullRotationRadians = 2 * Math.PI;
  public static final double maxSpeedMetersPerSec = 3.2;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.5);
  public static final double wheelBase = Units.inchesToMeters(22.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromDegrees(189.42);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromDegrees(102.23);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromDegrees(91.25);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromDegrees(13.42);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 33;
  public static final int backLeftDriveCanId = 35;
  public static final int frontRightDriveCanId = 23;
  public static final int backRightDriveCanId = 34;

  public static final int frontLeftTurnCanId = 38;
  public static final int backLeftTurnCanId = 25;
  public static final int frontRightTurnCanId = 24;
  public static final int backRightTurnCanId = 21;

  // Analog encoder ports
  public static final int frontLeftTurnEncId = 10;
  public static final int backLeftTurnEncId = 8;
  public static final int frontRightTurnEncId = 9;
  public static final int backRightTurnEncId = 11; // not connected atm

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction = 6.75;
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 21.13;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
