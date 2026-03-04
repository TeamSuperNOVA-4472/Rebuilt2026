package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.FieldMathHelpers;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetRobotRotation;
    private final Supplier<Double> mGetRobotAngularVelocity;
    private final BiConsumer<PoseEstimate, Matrix<N3,N1>> mUpdateRobotPose;

    // Limelight lib is stupid and wants integers for modes
    // Beat limelight's stupidity by creating an enum we can assign to a trigger
    private enum VisionMode {
        SEEDING(1), // Seeding means the limelights are calibrating themselves with the external gyro
        MIXED(4); // Mixed means the limelights are mixing their own IMU measurements with the external gyro

        private final int mMode;
        
        VisionMode(int pMode)
        {
            mMode = pMode;
        }

        public int get() { return mMode; }
    }

    public VisionSubsystem(Supplier<Double> pGetRobotRotation, Supplier<Double> pGetRobotAngularVelocity, BiConsumer<PoseEstimate, Matrix<N3,N1>> pUpdateRobotPose)
    {
        mGetRobotRotation = pGetRobotRotation; // Rotation supplies the heading of the robot at any point from -180 to 180
        mUpdateRobotPose = pUpdateRobotPose; // Allows us to pass the calculated pose to the drivetrain with estimated deviations
        mGetRobotAngularVelocity = pGetRobotAngularVelocity; // Supplies us the rotation velocity of the chassis

        setIMUMode(VisionMode.SEEDING); // Set initial mode to seed from gyro
    }

    private void updatePose(PoseEstimate pose)
    {
        // Update swerve subsystem with pose and standard deviations
        mUpdateRobotPose.accept(pose, calculateStdDevs(pose));
    }

    private void setIMUMode(VisionMode mode)
    {
        // Set the IMU to the correct mode for all limelights
        for (String limelight : Constants.VisionConstants.kLimelightNames)
        {
            LimelightHelpers.SetIMUMode(limelight, mode.get());
        }

        SmartDashboard.putString("Subsystems/VisionSubsystem/IMU Mode: ", mode.name());
    }

    private void setIMUThrottle(int throttle)
    {
        // Set the IMU's throttle for all limelights
        for (String limelight : Constants.VisionConstants.kLimelightNames)
        {
            LimelightHelpers.SetThrottle(limelight, throttle);
        }

        SmartDashboard.putNumber("Subsystems/VisionSubsystem/Throttle: ", throttle);
    }

    private void adjustThrottleAndIMU()
    {
        if (DriverStation.isEnabled())
        {
            setIMUMode(VisionMode.MIXED);
            setIMUThrottle(0);
        }
        else
        {
            setIMUMode(VisionMode.SEEDING);
            setIMUThrottle(VisionConstants.kThrottle);
        }
    }

    private Matrix<N3,N1> calculateStdDevs(PoseEstimate pose)
    {
        double lateraldev = pose.avgTagDist * Constants.VisionConstants.kBaseLateralDev; // Scale the standard deviation by tag distance
        double rotationaldev = mGetRobotAngularVelocity.get() * Constants.VisionConstants.kBaseRotDev; // Scale the rotational deviation by the angular velocity

        return VecBuilder.fill(lateraldev, lateraldev, rotationaldev);
    }

    private boolean underAmbiguityThreshold(PoseEstimate pose)
    {
        // Check through all tags seen
        for (RawFiducial id : pose.rawFiducials)
        {
            // Accept update if at least one tag has an ambiguity under the threshold
            SmartDashboard.putNumber("Subsystems/VisionSubsystem/Tag Ambiguity: ", id.ambiguity);
            if (id.ambiguity < Constants.VisionConstants.kAmbiguity) return true;
        }
        return false;
    }

    private boolean rejectUpdate(PoseEstimate pose)
    {
        // Check if the update passes all thresholds
        if (pose.avgTagDist <= Constants.VisionConstants.kTagDistThreshold && 
            pose.tagCount >= Constants.VisionConstants.kTagCountThreshold &&
            underAmbiguityThreshold(pose) &&
            mGetRobotAngularVelocity.get() < Constants.VisionConstants.kAngularVelocityThreshold)
        {
            return false;
        }
        return true;
    }

    //Estimate position of robot based off of limelight data
    private Optional<PoseEstimate> calculatePosition(String limelight)
    {
        boolean rejectUpdate = false;         
        LimelightHelpers.PoseEstimate pose;

        if(Constants.VisionConstants.kUseMegatag2)
        {
            //Localization--will not return location update if a Limelight can't see an Apriltag
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            
            SmartDashboard.putNumber("Subsystems/VisionSubsystem/Average Tag Distance: ", pose.avgTagDist);

            if(rejectUpdate(pose))
            {
                rejectUpdate = true;
            }
        }
        else
        {
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        }

        if(!rejectUpdate)
        {
            return Optional.of(pose); 
        }

        return Optional.empty();
    }

    // Calculate position, update position if present
    @Override
    public void periodic() {
        // Check all limelights active
        for(String limelight : Constants.VisionConstants.kLimelightNames)
        {
            // Set the robot orientation to the current heading (required for limelight's algorithm)
            LimelightHelpers.SetRobotOrientation(limelight, mGetRobotRotation.get(), 0, 0, 0, 0, 0);
            Optional<PoseEstimate> pose = calculatePosition(limelight); // Get update
            
            if(!pose.isEmpty()) // Update robot pose if all checks are passed
            {
                updatePose(pose.get());
                SmartDashboard.putString("Subsystems/VisionSubsystem/Pose: ", pose.get().pose.toString());
            }

            adjustThrottleAndIMU();
        }
    }
}


