package frc.robot;

import java.util.Optional;

import com.thethriftybot.server.msgHandler;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class FieldMathHelpers
{
    public enum Location{
        BUMP,
        TRENCH,
        NEUTRAL_ZONE_OR_OPPONENT,
        ALLIANCE_ZONE,
    }

    // TODO: add math for turret position offset from center, test equations for velocity, change flywheel from rpm to dist vs angle

    /**
     * Gets the distance in meters from a pose to the hub.
     * @param pose The bot pose.
     * @return Returns the distance in meters to the hub from the pose entered adjusted for the turret offset.
     */
    public static Translation2d getTranslationToHub(Pose2d pose)
    {
        Translation2d poseTranslation = pose.getTranslation();
        Translation2d hubPoseTranslation = getHubPose().getTranslation();

        return hubPoseTranslation.minus(poseTranslation);
    }

    /**
     * Calculates a "lead," or the desired heading to shoot at a constant speed from the robot when the robot is in motion.
     * The x and y velocities are field relative.
     * @param botPose The robot pose.
     * @param xVelocityMetersPerSecond The field-relative X velocity in meters per second.
     * @param yVelocityMetersPerSecond The field-relative Y velocity in meters per second.
     * @param projectileSpeed The constant projectile speed in meters per second.
     * @return The desired field relative heading from 0-360 where 0 is in line with the positive x axis.
     */
    private static Translation2d getTranslation2dToHubWithSomeSpeed(
        Pose2d botPose, 
        double xVelocityMetersPerSecond, 
        double yVelocityMetersPerSecond,
        double angularSpeedDegreesPerSecond)
    {
        // Calculate translations and distances
        Pose2d turretPose = botPose.transformBy(TurretConstants.kTurretOffset);
        Translation2d translationToHub = getTranslationToHub(turretPose);
        double distanceToHub = translationToHub.getNorm();
        double dt;

        Pair<Double, Double> fieldSpeeds = getFieldRelativeSpeedOfOffsetObject(normalizeDegrees(botPose.getRotation().getDegrees()), xVelocityMetersPerSecond, yVelocityMetersPerSecond, angularSpeedDegreesPerSecond);
        xVelocityMetersPerSecond = fieldSpeeds.getFirst();
        yVelocityMetersPerSecond = fieldSpeeds.getSecond();
        
        if (distanceToHub >= FlywheelConstants.kDistanceThresholdInMeters && distanceToHub <= FlywheelConstants.kDistanceMaximumInMeters)
        {
            dt = FlywheelConstants.kDistanceToFlywheelSpeedTime.get(distanceToHub) + VisionConstants.kLatencyLagInSeconds;
        } else {
            dt = 0;
        }
        // Calculate offsets
        double dx = xVelocityMetersPerSecond * dt;
        double dy = yVelocityMetersPerSecond * dt;
        Translation2d delta = new Translation2d(dx,dy);

        // Calculate adjusted translation
        Translation2d adjustedTranslation = translationToHub.minus(delta);
        return adjustedTranslation;
    }

    private static double getRotationToHubWithSomeSpeed(
        Pose2d botPose, 
        double xVelocityMetersPerSecond, 
        double yVelocityMetersPerSecond,
        double angularSpeedDegreesPerSecond)
    {
        return normalizeDegrees(getTranslation2dToHubWithSomeSpeed(botPose, xVelocityMetersPerSecond, yVelocityMetersPerSecond, angularSpeedDegreesPerSecond).getAngle().getDegrees());
    }

    public static double getRotationToPassOrShootWithSomeSpeed(
        Pose2d botPose, 
        double xVelocityMetersPerSecond, 
        double yVelocityMetersPerSecond,
        double angularSpeedDegreesPerSecond)
    {
        if (isInScoringZone(botPose))
        {
            return getRotationToHubWithSomeSpeed(botPose, xVelocityMetersPerSecond, yVelocityMetersPerSecond, angularSpeedDegreesPerSecond);
        }
        else
        {
            return isRedAlliance() ? 0 : 180;
        }
    }
    
    /**
     * Calculates the field relative speed of an object with a known offset from the robot center (i.e., the turret)
     * @param robotHeadingDegrees The current robot heading given in degrees.
     * @param xVelocityMetersPerSecond The current robot field-relative x velocity in meters per second.
     * @param yVelocityMetersPerSecond The current robot field-relative y velocity in meters per second.
     * @param angularSpeedDegreesPerSecond The current robot field-relative angular speed in degrees per second.
     * @param offset The robot-relative offset of the turret (or other object) from the center of the robot in meters.
     * @return The x and y field-relative velocities of the offset object in meters per second.
     */
    public static Pair<Double, Double> getFieldRelativeSpeedOfOffsetObject(
        double robotHeadingDegrees,
        double xVelocityMetersPerSecond, 
        double yVelocityMetersPerSecond, 
        double angularSpeedDegreesPerSecond)
    {
        double theta = Units.degreesToRadians(robotHeadingDegrees);
        double angularSpeed = Units.degreesToRadians(angularSpeedDegreesPerSecond); // This conversion works because the denominator doesn't change.

        // Take the cross product of the angular velocity and the offset and add to robot velocity vector, yay!
        // Troy (or some other smart person) check my math please
        double fieldRelativeXVelocity = xVelocityMetersPerSecond - angularSpeed * ((TurretConstants.kTurretOffset.getX() * Math.sin(theta)) + (TurretConstants.kTurretOffset.getY() * Math.cos(theta)));
        double fieldRelativeYVelocity = yVelocityMetersPerSecond + angularSpeed * ((TurretConstants.kTurretOffset.getX() * Math.cos(theta)) - (TurretConstants.kTurretOffset.getY() * Math.sin(theta)));

        return new Pair<Double, Double>(fieldRelativeXVelocity, fieldRelativeYVelocity);
    }

    public static double getVelocityTowardHub(Pose2d botPose, double xVelocityMetersPerSecond, double yVelocityMetersPerSecond, double angularSpeedDegreesPerSecond)
    {
        Pose2d turretPose = botPose.transformBy(TurretConstants.kTurretOffset);
        Translation2d unitVec = getTranslationToHub(turretPose);
        double dist = unitVec.getNorm();

        if (dist < 1e-6) return 0.0;

        unitVec = unitVec.div(dist);

        Pair<Double, Double> speeds = getFieldRelativeSpeedOfOffsetObject(normalizeDegrees(botPose.getRotation().getDegrees()), xVelocityMetersPerSecond, yVelocityMetersPerSecond, angularSpeedDegreesPerSecond);
        double xVel = speeds.getFirst();
        double yVel = speeds.getSecond();

        return xVel * unitVec.getX() + yVel * unitVec.getY();
    }

    /**
     * Checks if the robot is on the red alliance.
     * @return True if it is, false if it isn't or the alliance isn't valid.
     */
    private static boolean isRedAlliance()
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get().equals(Alliance.Red) ? true : false;
    }

    // Normalize from (-180, 180] to (0, 360)
    private static double normalizeDegrees(double degrees)
    {
        return (degrees % 360 + 360) % 360;
    }

    /**
     * Returns the correct hub pose.
     * @return a Pose2d with the hub pose.
     */
    private static Pose2d getHubPose()
    {
        // TODO: this is so ugly. I hate it. I'm going to make this look nice at some point. Maybe make it not hard coded somehow?
        if (isRedAlliance())
        {
            return Constants.VisionConstants.kIsAndyMark ? Constants.VisionConstants.kHubPoseRedAndyMarkMeters : Constants.VisionConstants.kHubPoseRedWeldedMeters;
        }
        else
        {
            return Constants.VisionConstants.kIsAndyMark ? Constants.VisionConstants.kHubPoseBlueAndyMarkMeters : Constants.VisionConstants.kHubPoseBlueWeldedMeters;
        }
    }

    public static Location getLocation(Pose2d botPose)
    {
        if (isUnderTrench(botPose))
        {
            return Location.TRENCH;
        }
        else if (isOnBump(botPose))
        {
            return Location.BUMP;
        }
        else if (isInScoringZone(botPose))
        {
            return Location.ALLIANCE_ZONE;
        }
        else
        {
            return Location.NEUTRAL_ZONE_OR_OPPONENT;
        }
    }

    private static Boolean isUnderTrench(Pose2d botPose)
    {
        double x = botPose.getX();
        double y = botPose.getY();

        if (((x < VisionConstants.kRedTrenchXHighThreshold && x > VisionConstants.kRedTrenchXLowThreshold) ||
            (x < VisionConstants.kBlueTrenchXHighThreshold && x > VisionConstants.kBlueTrenchXLowThreshold)) &&
            (y > VisionConstants.kTopTrenchYThreshold || y < VisionConstants.kBottomTrenchYThreshold))
        {
            return true;
        }
        return false;
    }

    private static Boolean isOnBump(Pose2d botPose)
    {
        double x = botPose.getX();
        double y = botPose.getY();

        if (((x > VisionConstants.kRedBumpXLowThreshold && x < VisionConstants.kRedBumpXHighThreshold) ||
            (x > VisionConstants.kBlueBumpXLowThreshold && x < VisionConstants.kBlueBumpXHighThreshold)) &&
            (y < VisionConstants.kTopTrenchYThreshold && y > VisionConstants.kBottomTrenchYThreshold))
        {
            return true;
        }
        return false;
    }

    private static boolean isInScoringZone(Pose2d botPose)
    {
        if (isRedAlliance())
        {
            return botPose.getX() > VisionConstants.kRedBumpXLowThreshold ? true : false;
        }
        else
        {
            return botPose.getX() < VisionConstants.kBlueBumpXHighThreshold ? true : false;
        }
    }
}
