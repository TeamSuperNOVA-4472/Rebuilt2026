package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.Commands.ResetCommands.resetTurretEncoder;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetRobotRotation;
    private Pose2d mLastValidPose = new Pose2d();
    private final Supplier<Double> mGetRobotAngularVelocity;
    private final BiConsumer<PoseEstimate, Matrix<N3,N1>> mUpdateRobotPose;
    private final Field2d mField;
    private ArrayList<Integer> mRestrictedTags = VisionConstants.kHubTags;
    private boolean mUseMegaTag2 = VisionConstants.kUseMegatag2ByDefault;
    private boolean mRestrictTags = VisionConstants.kRestrictTagsByDefault;

    // Limelight lib is stupid and wants integers for modes
    // Beat limelight's stupidity by creating an enum we can assign to a trigger
    private enum VisionMode {
        SEEDING(1), // Seeding means the limelights are calibrating themselves with the external gyro
        MIXED(0); // Mixed means the limelights are mixing their own IMU measurements with the external gyro

        private final int mMode;
        
        VisionMode(int pMode)
        {
            mMode = pMode;
        }

        public int get() { return mMode; }
    }

    public VisionSubsystem(
        Supplier<Double> pGetRobotRotation, 
        Supplier<Double> pGetRobotAngularVelocity, 
        BiConsumer<PoseEstimate, Matrix<N3,N1>> pUpdateRobotPose)
    {
        mGetRobotRotation = pGetRobotRotation; // Rotation supplies the heading of the robot at any point from -180 to 180
        mUpdateRobotPose = pUpdateRobotPose; // Allows us to pass the calculated pose to the drivetrain with estimated deviations
        mGetRobotAngularVelocity = pGetRobotAngularVelocity; // Supplies us the rotation velocity of the chassis

        setIMUMode(VisionMode.SEEDING); // Set initial mode to seed from gyro

        mField = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", mField);
    }

    public void disableMT2() { mUseMegaTag2 = false; }
    public void enableMT2() { mUseMegaTag2 = true; }

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

    @AutoLogOutput
    public Pose2d getLastValidPose()
    {
        return mLastValidPose;
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
        if (!mUseMegaTag2) lateraldev *= VisionConstants.kMegaTag1Multiplier;
        if (mRestrictTags) lateraldev *= VisionConstants.kRestrictedTagsMultiplier;
        double rotationaldev = mUseMegaTag2 ? Double.POSITIVE_INFINITY : pose.avgTagDist * Constants.VisionConstants.kBaseRotDev ; // If MT1, scale by distance and square

        return VecBuilder.fill(lateraldev, lateraldev, rotationaldev);
    }

    private boolean underAmbiguityThreshold(PoseEstimate pose)
    {
        if (pose.rawFiducials[0].ambiguity > VisionConstants.kAmbiguity && pose.tagCount == 1)
        {
        return false; 
        }
        return true;
    }

    private boolean rejectUpdate(PoseEstimate pose)
    {
        // Check if the update passes all thresholds
        if (pose.avgTagDist <= Constants.VisionConstants.kTagDistThreshold && 
            pose.tagCount >= Constants.VisionConstants.kTagCountThreshold &&
            pose.pose.getX() >= 0 &&
            pose.pose.getY() >= 0 &&
            pose.pose.getX() <= AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getFieldLength() &&
            pose.pose.getY() <= AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getFieldWidth() &&
            mGetRobotAngularVelocity.get() <= Constants.VisionConstants.kAngularVelocityThreshold &&
            underAmbiguityThreshold(pose) &&
            withinAcceptedTags(pose))
        {
            return false;
        }
        return true;
    }

    private boolean withinAcceptedTags(PoseEstimate pose)
    {
        if (!mRestrictTags) return true;

        for (RawFiducial id : pose.rawFiducials)
        {
            if (!mRestrictedTags.contains(id.id)) return false;
        }

        return true;
    }

    // public void restrictToClimbTags() { mRestrictedTags = VisionConstants.kClimbTags; }
    // public void restrictToHubTags() { mRestrictedTags = VisionConstants.kHubTags; }
  
    //Estimate position of robot based off of limelight data
    private Optional<PoseEstimate> calculatePosition(String limelight)
    {
        boolean rejectUpdate = false;         
        LimelightHelpers.PoseEstimate pose;

        if(mUseMegaTag2)
        {
            //Localization--will not return location update if a Limelight can't see an Apriltag
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
        }
        else
        {
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        }

        if(rejectUpdate(pose))
        {
            rejectUpdate = true;
        }

        SmartDashboard.putNumber("Subsystems/VisionSubsystem/Average Tag Distance: ", pose.avgTagDist);
       

        if(!rejectUpdate)
        {
            return Optional.of(pose); 
        }

        return Optional.empty();
    }

    @AutoLogOutput(key = "VisionSubsystem/LogPose")
    private Pose2d logPose = new Pose2d();
    
    

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
                mLastValidPose = pose.get().pose;
                SmartDashboard.putString("Subsystems/VisionSubsystem/Pose: ", pose.get().pose.toString());
            }

            adjustThrottleAndIMU();
        }
    }
}




