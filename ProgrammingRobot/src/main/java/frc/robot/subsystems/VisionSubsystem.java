package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetRobotRotation;
    private final Consumer<PoseEstimate> mUpdateRobotPose;

    private final Trigger mRobotEnabled;

    private final Field2d mField;

    private enum VisionMode {
        SEEDING(1),
        MIXED(4);

        private final int mMode;
        
        VisionMode(int pMode)
        {
            mMode = pMode;
        }

        public int get() { return mMode; }
    }

    public VisionSubsystem(Supplier<Double> pGetRobotRotation, Consumer<PoseEstimate> pUpdateRobotPose)
    {
        mGetRobotRotation = pGetRobotRotation;
        mUpdateRobotPose = pUpdateRobotPose;

        mRobotEnabled = new Trigger(RobotState::isEnabled);

        mField = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", mField);

        setIMUMode(VisionMode.SEEDING);

        mRobotEnabled.onTrue(mixLimelightIMU());
        mRobotEnabled.onFalse(seedLimelightIMU());
    }

    private void updatePose(PoseEstimate pose)
    {
        mUpdateRobotPose.accept(pose);
    }

    private void setIMUMode(VisionMode mode)
    {
        for (String limelight : Constants.VisionConstants.kLimelightNames)
        {
            LimelightHelpers.SetIMUMode(limelight, mode.get());
        }
    }

    //Estimate position of robot based off of limelight data
    private Optional<PoseEstimate> calculatePosition(String limelight)
    {
        boolean rejectUpdate = false;         
        LimelightHelpers.PoseEstimate pose;

        // TODO: Add and test filters for MegaTag2
        if(Constants.VisionConstants.kUseMegatag2)
        {
            //Localization--will not return location update if a Limelight can't see an Apriltag
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            if(pose.tagCount == 0||pose.rawFiducials[0].ambiguity >= Constants.VisionConstants.kAmbiguity)
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

    // Public interfaces
    public Command seedLimelightIMU() { return new InstantCommand(() -> setIMUMode(VisionMode.SEEDING)); } 
    public Command mixLimelightIMU() { return new InstantCommand(() -> setIMUMode(VisionMode.MIXED)); }

    //Calculate position, update position if present
    @Override
    public void periodic() {
        for(String limelight : Constants.VisionConstants.kLimelightNames)
        {
            LimelightHelpers.SetRobotOrientation(limelight, mGetRobotRotation.get(), 0, 0, 0, 0, 0);
            Optional<PoseEstimate> pose = calculatePosition(limelight);
            if(!pose.isEmpty()) 
            {
                updatePose(pose.get());            
            }
                            
        }
    }
}


