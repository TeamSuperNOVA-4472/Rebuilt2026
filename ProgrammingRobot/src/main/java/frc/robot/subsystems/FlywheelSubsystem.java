package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    public static final FlywheelSubsystem kFlywheel = new FlywheelSubsystem();
    public enum FlywheelMode{
        OFF,
        SPINNING    
    }
    private SparkMax kFlywheelMotor;
    private PIDController kHoodPidController;
    private FlywheelMode kMode;

    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        kFlywheelMotor = new SparkMax(9, MotorType.kBrushless);
    }
    private void moveFlywheel(){
        switch (kMode) {
        case OFF:
            kFlywheelMotor.set(0);
            break;
        
        case SPINNING:
            kFlywheelMotor.set(0.8);
            break;
        }
    }
    public double getSpinSpeed(){
        return kFlywheelMotor.get();
    }
    public FlywheelMode getMode(){
        return kMode;
    }
    public void setMode(FlywheelMode mNewMode){
        kMode = mNewMode;
        moveFlywheel();
    }
}