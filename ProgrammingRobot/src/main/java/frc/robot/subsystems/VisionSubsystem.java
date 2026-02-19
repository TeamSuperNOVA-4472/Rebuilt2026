package frc.robot.subsystems;

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
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetRobotRotation;
    private final Supplier<Double> mGetRobotAngularVelocity;
    private final BiConsumer<PoseEstimate, Matrix<N3,N1>> mUpdateRobotPose;

    private final Trigger mRobotEnabled;

    private final Field2d mField;

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

        mRobotEnabled = new Trigger(RobotState::isEnabled); // Trigger to check if robot is enabled

        mField = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", mField);

        setIMUMode(VisionMode.SEEDING); // Set initial mode to seed from gyro

        mRobotEnabled.onTrue(mixLimelightIMU().andThen(removeThrottle())); // When robot is enabled, remove throttle and mix IMU measurements
        mRobotEnabled.onFalse(seedLimelightIMU().andThen(setThrottle())); // When robot is disabled, add throttle and seed IMU from gyro

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
    }

    private void setIMUThrottle(int throttle)
    {
        // Set the IMU's throttle for all limelights
        for (String limelight : Constants.VisionConstants.kLimelightNames)
        {
            LimelightHelpers.SetThrottle(limelight, throttle);
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
            if (id.ambiguity < Constants.VisionConstants.kAmbiguity) return true;
        }
        return false;
    }

    private boolean rejectUpdate(PoseEstimate pose)
    {
        // Check if the update passes all thresholds
        if (pose.avgTagDist <= Constants.VisionConstants.kTagDistThreshold && 
            pose.tagCount >= Constants.VisionConstants.kTagCountThreshold &&
            underAmbiguityThreshold(pose))
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

    // Command interfaces
    private Command seedLimelightIMU() { return new InstantCommand(() -> setIMUMode(VisionMode.SEEDING)); } 
    private Command mixLimelightIMU() { return new InstantCommand(() -> setIMUMode(VisionMode.MIXED)); }

    private Command setThrottle() { return new InstantCommand(() -> setIMUThrottle(Constants.VisionConstants.kThrottle)); }
    private Command removeThrottle() { return new InstantCommand(() -> setIMUThrottle(0)); }

    // Calculate position, update position if present
    @Override
    public void periodic() {
        // Check all limelights active
        for(String limelight : Constants.VisionConstants.kLimelightNames)
        {
            // TODO: does giving angular velocity to this make measurements more consistent?
            // Set the robot orientation to the current heading (required for limelight's algorithm)
            LimelightHelpers.SetRobotOrientation(limelight, mGetRobotRotation.get(), 0, 0, 0, 0, 0);
            Optional<PoseEstimate> pose = calculatePosition(limelight); // Get update

            if(!pose.isEmpty()) // Update robot pose if all checks are passed
            {
                updatePose(pose.get());            
            }
                            
        }
    }
}


