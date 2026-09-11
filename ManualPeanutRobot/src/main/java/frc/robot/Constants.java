// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MAXIMUM_SPEED_METERS_PER_SECONDS = 4.5;
    public static final double METERS_PER_INCH = 0.0254;
    public static final double SWERVE_LOC_Y_INCHES = 7.5;
    public static final double SWERVE_LOC_X_INCHES = 7;
    public static final double SWERVE_LOC_Y_METERS = SWERVE_LOC_Y_INCHES * METERS_PER_INCH;
    public static final double SWERVE_LOC_X_METERS = SWERVE_LOC_X_INCHES * METERS_PER_INCH;

    public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
    public static final int FRONT_LEFT_TURN_CAN_ID = 2;
    public static final int FRONT_LEFT_TURN_PWM_PORT = 0;
    public static final int FRONT_LEFT_TURN_ABS_ENC_PORT = 0;
    public static final double FRONT_LEFT_TURN_ABS_ENC_OFFSET = 55;
    public static final int FRONT_LEFT_TURN_REL_ENC_PORT_A = 0;
    public static final int FRONT_LEFT_TURN_REL_ENC_PORT_B = 1;

    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 3;
    public static final int FRONT_RIGHT_TURN_CAN_ID = 4;
    public static final int FRONT_RIGHT_TURN_PWM_PORT = 1;
    public static final int FRONT_RIGHT_TURN_ABS_ENC_PORT = 1;
    public static final double FRONT_RIGHT_TURN_ABS_ENC_OFFSET = -117;
    public static final int FRONT_RIGHT_TURN_REL_ENC_PORT_A = 2;
    public static final int FRONT_RIGHT_TURN_REL_ENC_PORT_B = 3;

    public static final int REAR_LEFT_DRIVE_CAN_ID = 5;
    public static final int REAR_LEFT_TURN_CAN_ID = 6;
    public static final int REAR_LEFT_TURN_PWM_PORT = 2;
    public static final int REAR_LEFT_TURN_ABS_ENC_PORT = 2;
    public static final double REAR_LEFT_TURN_ABS_ENC_OFFSET = -125;
    public static final int REAR_LEFT_TURN_REL_ENC_PORT_A = 4;
    public static final int REAR_LEFT_TURN_REL_ENC_PORT_B = 5;

    public static final int REAR_RIGHT_DRIVE_CAN_ID = 7;
    public static final int REAR_RIGHT_TURN_CAN_ID = 8;
    public static final int REAR_RIGHT_TURN_PWM_PORT = 3;
    public static final int REAR_RIGHT_TURN_ABS_ENC_PORT = 3;
    public static final double REAR_RIGHT_TURN_ABS_ENC_OFFSET = -30;
    public static final int REAR_RIGHT_TURN_REL_ENC_PORT_A = 6;
    public static final int REAR_RIGHT_TURN_REL_ENC_PORT_B = 7;

    public static final double TURN_TICS_PER_ROTATION = 415.1;
    public static final double DRIVE_TICS_PER_ROTATION = 6830;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4;
    public static final double DRIVE_GEAR_REDUCTION = 6.67;
    public static final double DRIVE_DISTANCE_PER_NEO_ROTATION_METERS = (DRIVE_WHEEL_DIAMETER_INCHES * Math.PI * METERS_PER_INCH) / DRIVE_GEAR_REDUCTION;
    public static final double DRIVE_DISTANCE_PER_TIC_METERS = (DRIVE_WHEEL_DIAMETER_INCHES * Math.PI * METERS_PER_INCH) / DRIVE_TICS_PER_ROTATION;
    public static final double METERS_PER_SECOND_TO_DRIVE_SPEED_TICS = 1 / (DRIVE_DISTANCE_PER_TIC_METERS * 10);

    public static final double TURN_P = 0.01;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.0001;

    public static final double DRIVE_P = 0.25; 
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0; 
    public static final double DRIVE_F = 0;

    public static final double AUTOALIGN_P = 0.15;
    public static final double AUTOALIGN_D = 0.002;

    public static final double SWERVE_RADIUS_INCHES = Math.sqrt(Math.pow(SWERVE_LOC_X_INCHES, 2) + Math.pow(SWERVE_LOC_Y_INCHES, 2));
    public static final double SWERVE_CIRCUMFERENCE_METERS = 2 * Math.PI * SWERVE_RADIUS_INCHES * METERS_PER_INCH; //1.6372863352652361048052029816421;
    public static final double METERS_PER_SECONDS_TO_RADIANS_PER_SECONDS = (2 * Math.PI) / SWERVE_CIRCUMFERENCE_METERS;
    public static final double METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS = (360) / SWERVE_CIRCUMFERENCE_METERS;
    public static final double GYRO_ERROR_FACTOR = 1.0;

    public static final double DRIVE_KS = 1.0833 / 10;
    public static final double DRIVE_KV = 2.9124 / 10;
    public static final double DRIVE_KA = 0.48167 / 10;

    public static final double ROBOT_SIZE_OFFSET = Units.inchesToMeters(22.5);

    public static final InterpolatingTreeMap<Double,Double> CONTROLLER_PROFLE_MAP = new InterpolatingTreeMap<>();
    public static final InterpolatingTreeMap<Double,Double> LIMELIGHT_DISTANCE_MAP = new InterpolatingTreeMap<>();
    static {
        CONTROLLER_PROFLE_MAP.put(0.0, 0.0);
        CONTROLLER_PROFLE_MAP.put(0.8, 1.0/3.0);
        CONTROLLER_PROFLE_MAP.put(1.0, 1.0);

        /* Limelight values */
        LIMELIGHT_DISTANCE_MAP.put(0.045, Units.inchesToMeters(51));
        LIMELIGHT_DISTANCE_MAP.put(0.084, Units.inchesToMeters(34));
        LIMELIGHT_DISTANCE_MAP.put(0.121, Units.inchesToMeters(17));
        LIMELIGHT_DISTANCE_MAP.put(.240, Units.inchesToMeters(0));

    }
}
