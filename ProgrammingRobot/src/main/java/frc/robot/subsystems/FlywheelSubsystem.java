package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
    private SparkMax kFlywheelMotor;
    private FlywheelMode kMode;
    private double kTargetSpeed;
    private SimpleMotorFeedforward kFlywheelFeedforward;
    private PIDController kFlywheelFeedback;

    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheelMotor = new SparkMax(33, MotorType.kBrushless);
        kFlywheelFeedforward = new SimpleMotorFeedforward(0.0001, 0.0075);
        kFlywheelFeedback = new PIDController(0.13,0, 0.001);
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
    @Override
    public void periodic(){
        kFlywheelMotor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheelMotor.getEncoder().getVelocity()/6000, kTargetSpeed) + kFlywheelFeedforward.calculate(kFlywheelMotor.getEncoder().getVelocity()), -11, 11));
    }

}