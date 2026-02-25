package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private TalonFX kFlywheel1Motor;
    private TalonFX kFlywheel2Motor;
    private TalonFX kFlywheelHoodMotor;
    private PIDController kHoodPidController;
    private FlywheelMode kMode;
    private double kTargetAngle;
    private double kTargetSpeed;
    private boolean kHoodAtTarget;
    private SimpleMotorFeedforward kFlywheel1Feedforward;
    private SimpleMotorFeedforward kFlywheel2Feedforward;
    private PIDController kFlywheelFeedback;

    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheel1Motor = new TalonFX(7);
        kFlywheel2Motor = new TalonFX(33);
        kFlywheelHoodMotor = new TalonFX(6);
        kHoodPidController = new PIDController(0,0,0);
        kFlywheel1Feedforward = new SimpleMotorFeedforward(0.0001, 0.0075);
        kFlywheel2Feedforward = new SimpleMotorFeedforward(0.0001, 0.0075);
        kFlywheelFeedback = new PIDController(0.13,0, 0.001);
        kTargetAngle = 90;

        TalonFXConfiguration kFlywheel1Config = new TalonFXConfiguration();
        CurrentLimitsConfigs kFlywheel1CurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kFlywheel1MotorConfig = new MotorOutputConfigs();
        kFlywheel1Motor.getConfigurator().refresh(kFlywheel1Config);
        kFlywheel1Motor.getConfigurator().refresh(kFlywheel1CurrentConfig);
        kFlywheel1Motor.getConfigurator().refresh(kFlywheel1MotorConfig);
        kFlywheel1CurrentConfig.SupplyCurrentLimit = 40;
        kFlywheel1CurrentConfig.SupplyCurrentLimitEnable = true;
        kFlywheel1CurrentConfig.StatorCurrentLimitEnable = true;
        kFlywheel1CurrentConfig.StatorCurrentLimit = 40;
        kFlywheel1MotorConfig.NeutralMode = NeutralModeValue.Coast;
        kFlywheel1Config.withCurrentLimits(kFlywheel1CurrentConfig);
        kFlywheel1Config.withMotorOutput(kFlywheel1MotorConfig);
        kFlywheel1Motor.getConfigurator().apply(kFlywheel1Config);

        TalonFXConfiguration kFlywheel2Config = new TalonFXConfiguration();
        CurrentLimitsConfigs kFlywheel2CurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kFlywheel2MotorConfig = new MotorOutputConfigs();
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2Config);
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2CurrentConfig);
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2MotorConfig);
        kFlywheel2CurrentConfig.SupplyCurrentLimit = 40;
        kFlywheel2CurrentConfig.SupplyCurrentLimitEnable = true;
        kFlywheel2CurrentConfig.StatorCurrentLimitEnable = true;
        kFlywheel2CurrentConfig.StatorCurrentLimit = 40;
        kFlywheel2MotorConfig.NeutralMode = NeutralModeValue.Coast;
        kFlywheel2Config.withCurrentLimits(kFlywheel2CurrentConfig);
        kFlywheel2Config.withMotorOutput(kFlywheel2MotorConfig);
        kFlywheel2Motor.getConfigurator().apply(kFlywheel2Config);

        TalonFXConfiguration kFlywheelHoodConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kFlywheelHoodCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kFlywheelHoodMotorConfig = new MotorOutputConfigs();
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodConfig);
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodCurrentConfig);
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodMotorConfig);
        kFlywheelHoodCurrentConfig.SupplyCurrentLimit = 2;
        kFlywheelHoodCurrentConfig.SupplyCurrentLimitEnable = true;
        kFlywheelHoodCurrentConfig.StatorCurrentLimitEnable = true;
        kFlywheelHoodCurrentConfig.StatorCurrentLimit = 2;
        kFlywheelHoodMotorConfig.NeutralMode = NeutralModeValue.Brake;
        kFlywheelHoodConfig.withCurrentLimits(kFlywheelHoodCurrentConfig);
        kFlywheelHoodConfig.withMotorOutput(kFlywheelHoodMotorConfig);
        kFlywheelHoodMotor.getConfigurator().apply(kFlywheelHoodConfig);
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
        return kFlywheel1Motor.get();
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

    //TODO: turn encoder angle to hood angle 
    public boolean getHoodAtTartget(){
        return kHoodAtTarget;
    }
    @Override
    public void periodic(){
        kHoodAtTarget = kHoodPidController.atSetpoint();
        kFlywheelHoodMotor.set(kHoodPidController.calculate(kFlywheelHoodMotor.getPosition().getValueAsDouble(),kTargetAngle));
        kFlywheel1Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel1Motor.getVelocity().getValueAsDouble()/512.0, kTargetSpeed) + kFlywheel1Feedforward.calculate(kFlywheel1Motor.getVelocity().getValueAsDouble()), -11, 11));
        kFlywheel2Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel2Motor.getVelocity().getValueAsDouble()/512.0, kTargetSpeed) + kFlywheel2Feedforward.calculate(kFlywheel2Motor.getVelocity().getValueAsDouble()), -11, 11));
    }

}