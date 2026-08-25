package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTableEntry mDeltaX;
    private NetworkTableEntry mDeltaY;
    private NetworkTableEntry mTargetArea;
    private NetworkTableEntry mTargetsPresent;
    /*
     * TableEntry values:
     *  tx -> The horizontal offset from the crosshair to the vision target
     *  ty -> The vertical offset from the crosshair to the vision target
     *  ta -> The target area / percentage that the target takes up of the image
     *  tv -> A binary value representing if the Limelight has any valid targets
     */

    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-nova");
        mDeltaX = table.getEntry("tx"); 
        mDeltaY = table.getEntry("ty");
        mTargetArea = table.getEntry("ta");
        mTargetsPresent = table.getEntry("tv");
    }

    public double getXDegsFromTarget() {
        return mDeltaX.getDouble(0);
    }

    public double getYDegsFromTarget() {
        return mDeltaY.getDouble(0);
    }


    /*
     * Returns either true or false depending on the value of the tv entry
     * 0 -> No valid targets discovered
     * 1 -> Valid target is present
     */
    public boolean targetsPresent() {
        return mTargetsPresent.getDouble(0) > 0;
    }
    
    public double getTargetArea() {
        return mTargetArea.getDouble(0);
    }

    /*
     * Get the angle relative to the target
     * 
     *  @return the current drivetrain rotation plus the tx entry value
     */
    public double getAngleToTarget() {
        return mDeltaX.getDouble(0);
    }
}