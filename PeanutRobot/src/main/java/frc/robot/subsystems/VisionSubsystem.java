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
import frc.robot.LimelightHelpers.RawFiducial;


public class VisionSubsystem extends SubsystemBase
{
    //Suppliers and constants, members of class
    private final Supplier<Double> mGetGyroPosition;
    private static final boolean kUseMegatag2 = false; 
    private static final String[] kLimelightNames = {"limelight"};
    private final ArrayList<Consumer<PoseEstimate>> mListeners;
    private final Field2d mField; 

    
    public VisionSubsystem(Supplier<Double> pGetGyroPosition)
    {
        mGetGyroPosition = pGetGyroPosition;
        mField = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", mField);
        mListeners = new ArrayList<>();
    
    }
    public void addMeasurementListener(Consumer<PoseEstimate> consumer)
    {
        mListeners.add(consumer);
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
    public Optional<PoseEstimate> calculatePosition(String limelight)
    {
        boolean rejectUpdate = false;         
        LimelightHelpers.PoseEstimate pose; 
        if(kUseMegatag2)
        {
            //Localization--will not return location update if a Limelight can't see an Apriltag
            LimelightHelpers.SetRobotOrientation(limelight, mGetGyroPosition.get(), 0, 0, 0, 0, 0);
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            //Tester for the rejectUpdate function from the programming robot.
            rejectUpdate = rejectUpdate(pose);
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
        for(String limelight : kLimelightNames)
        {
            Optional<PoseEstimate> pose = calculatePosition(limelight);
            if(!pose.isEmpty()) 
            {
                updatePose(pose.get());
                SmartDashboard.putString("Vision position",pose.get().pose.toString());
            
            }
                            
        }

        
        SmartDashboard.putNumber("Tag info", LimelightHelpers.getTX("limelight"));
    }
}


