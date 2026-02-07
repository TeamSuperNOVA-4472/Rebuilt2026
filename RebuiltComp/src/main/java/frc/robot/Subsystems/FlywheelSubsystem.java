package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    public static final FlywheelSubsystem kFlywheel = new FlywheelSubsystem();

    public enum FlywheelMode{
        OFF,
        SPINNING    
    }
    private TalonFX kFlywheelMotor;
    private TalonFX kFlywheelHoodMotor;
    private PIDController kHoodPidController;
    private FlywheelMode kMode;
    private double kTargetAngle;
    private double kTargetSpeed;
    private boolean kHoodAtTarget;
    private SimpleMotorFeedforward kFlywheelFeedforward;
    private PIDController kFlywheelFeedback;

    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheelMotor = new TalonFX(7);
        kFlywheelHoodMotor = new TalonFX(6);
        kHoodPidController = new PIDController(0,0,0);
        kFlywheelFeedforward = new SimpleMotorFeedforward(0.0001, 0.0075);
        kFlywheelFeedback = new PIDController(0.13,0, 0.001);
        kTargetAngle = 90;
    }
    private void moveFlywheel(){
        switch (kMode) {
        case OFF:
            kTargetSpeed = 0;
            break;
        
        case SPINNING:
            kTargetSpeed = 0.8;
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
    public boolean getHoodAtTartget(){
        return kHoodAtTarget;
    }
    @Override
    public void periodic(){
        kHoodAtTarget = kHoodPidController.atSetpoint();
        kFlywheelHoodMotor.set(kHoodPidController.calculate(kFlywheelHoodMotor.getPosition().getValueAsDouble(),kTargetAngle));
        kFlywheelMotor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheelMotor.getVelocity().getValueAsDouble()/512.0, kTargetSpeed) + kFlywheelFeedforward.calculate(kFlywheelMotor.getVelocity().getValueAsDouble()), -11, 11));
    }

}