package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    public enum FlywheelMode{
        OFF,
        SPINNING    
    }
    private TalonFX kFlywheelMotor;
    private TalonFX kFlywheelHoodMotor;
    private PIDController kHoodPidController;
    private FlywheelMode kMode;
    private double kTargetAngle;

    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        kFlywheelMotor = new TalonFX(7);
        kFlywheelHoodMotor = new TalonFX(6);
        kHoodPidController = new PIDController(0,0,0);
        kTargetAngle = 90;
    }
    private void moveFlywheel(){
        switch (kMode) {
        case OFF:
            kFlywheelMotor.set(0);
            break;
        
        case SPINNING:
            kFlywheelMotor.set(1);
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
    public void setHoodTarget(double mNewTarget){
        kTargetAngle = mNewTarget;
    }
    @Override
    public void periodic(){
        kFlywheelHoodMotor.set(kHoodPidController.calculate(kFlywheelHoodMotor.getPosition().getValueAsDouble(),kTargetAngle));
    }
}
