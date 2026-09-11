package frc.robot.util;

import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public final class DynamicTuner {
    static DynamicTuner singleton_instance = null;
    static ShuffleboardTab tuningTab;
    
    static ShuffleboardLayout autoAligntuningList;
    static GenericEntry drive_kP;
    static GenericEntry drive_kD;

    public void initialize() {
        tuningTab = Shuffleboard.getTab("TuningTab");
        
        /*
         * Auto-alignment constants
         */

        autoAligntuningList = tuningTab.getLayout("Auto alignment PID Tuning", BuiltInLayouts.kList);

        drive_kP = autoAligntuningList.add("autoAlign_kP", Constants.AUTOALIGN_P).getEntry();
        drive_kD = autoAligntuningList.add("autoAlign_kD", Constants.AUTOALIGN_D).getEntry();
    }

    public double getAutoAlignP() {
        return drive_kP.getDouble(Constants.AUTOALIGN_P);
    }

    public double getAutoAlignD() {
        return drive_kD.getDouble(Constants.AUTOALIGN_D);
    }

    public static DynamicTuner getInstance() {
        if (singleton_instance == null)
            singleton_instance = new DynamicTuner();
        
        return singleton_instance;
    }
}