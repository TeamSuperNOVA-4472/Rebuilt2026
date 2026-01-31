package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetGyroPosition;

    private final ArrayList<Consumer<PoseEstimate> > mListeners;
    private final Field2d mField; 

    //Get gyro position 
    public VisionSubsystem(Supplier<Double> pGetGyroPosition)
    {
        mGetGyroPosition = pGetGyroPosition;
        mField = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", mField);
        mListeners = new ArrayList<>();
        LimelightHelpers.SetIMUMode("limelight", 4);
    
    }
    public void addMeasurementListener(Consumer<PoseEstimate> consumer)
    {
        mListeners.add(consumer);
    }

    //Estimate position of robot based off of limelight data
    public Optional<PoseEstimate> calculatePosition(String limelight)
    {
        boolean rejectUpdate = false;         
        LimelightHelpers.PoseEstimate pose; 
        if(Constants.VisionConstants.kUseMegatag2)
        {
            //Localization--will not return location update if a Limelight can't see an Apriltag
            LimelightHelpers.SetRobotOrientation(limelight, mGetGyroPosition.get(), 0, 0, 0, 0, 0);
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            /*if(pose.tagCount == 0)
            {
                rejectUpdate = true;
            }*/
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

    private void updatePose(PoseEstimate pose)
    {
        for(Consumer<PoseEstimate> callback : mListeners) callback.accept(pose);
    } 

    //Calculate position, update position if present
    @Override
    public void periodic() {
        for(String limelight : Constants.VisionConstants.kLimelightNames)
        {
            Optional<PoseEstimate> pose = calculatePosition(limelight);
            if(!pose.isEmpty()) 
            {
                updatePose(pose.get());
                SmartDashboard.putString("Vision position",pose.get().pose.toString());
            
            }
                            
        }

        SmartDashboard.putNumber("Gyro position: ", mGetGyroPosition.get());
        SmartDashboard.putNumber("Tag info", LimelightHelpers.getTX("limelight"));
    }
}


