package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX kClimbMotor;
    private ClimbState kState = ClimbState.STORED;
    private PIDController kClimbController;

    public enum ClimbState{
        STORED,
        UP,
        CLIMB
    }
    private boolean kAtSetpoint = false;
    private double kTarget = 0;

    public ClimbSubsystem()
    {
        kClimbMotor = new TalonFX(ClimbConstants.kClimbMotorPort, ClimbConstants.kClimbCanbus);

        TalonFXConfiguration kClimbConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kClimbCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kClimbMotorConfig = new MotorOutputConfigs();
        kClimbController = new PIDController(0, 0, 0);
        kClimbMotor.getConfigurator().refresh(kClimbConfig);
        kClimbMotor.getConfigurator().refresh(kClimbCurrentConfig);
        kClimbMotor.getConfigurator().refresh(kClimbMotorConfig);
        kClimbCurrentConfig.SupplyCurrentLimit = ClimbConstants.kClimbSupplyLimit;
        kClimbCurrentConfig.SupplyCurrentLimitEnable = ClimbConstants.kClimbSupplyLimitEnabled;
        kClimbCurrentConfig.StatorCurrentLimitEnable = ClimbConstants.kClimbStatorLimitEnabled;
        kClimbCurrentConfig.StatorCurrentLimit = ClimbConstants.kClimbStatorLimit;
        kClimbCurrentConfig.SupplyCurrentLowerLimit = ClimbConstants.kClimbSupplyLimit;
        kClimbMotorConfig.NeutralMode = ClimbConstants.kClimbNeutralMode;
        kClimbConfig.withCurrentLimits(kClimbCurrentConfig);
        kClimbConfig.withMotorOutput(kClimbMotorConfig);
        kClimbMotor.getConfigurator().apply(kClimbConfig);
    }

    public void setVoltage(double volts)
    {
        kClimbMotor.setVoltage(volts);
    }

    public void setState(ClimbState mState){
        kState = mState;
        moveWithState();
    }
    @Override
    public void periodic(){
        kAtSetpoint = kClimbController.atSetpoint();
        if (kState == ClimbState.CLIMB && !kAtSetpoint){
            setVoltage(MathUtil.clamp(kClimbController.calculate(kClimbMotor.getPosition().getValueAsDouble() * ClimbConstants.kEncoderToInches, kTarget) - ClimbConstants.kClimbVoltage,-10, 10));
        }
        else if(!kAtSetpoint){
            setVoltage(MathUtil.clamp(kClimbController.calculate(kClimbMotor.getPosition().getValueAsDouble() * ClimbConstants.kEncoderToInches, kTarget),-10, 10));
        }
        else{
            setVoltage(0);
        }
    }
    private void moveWithState(){
        switch (kState) {
            case STORED:
                kTarget = ClimbConstants.kStored;
                break;
        
            case UP:
                kTarget = ClimbConstants.kUP;
                break;

            case CLIMB:
                kTarget = ClimbConstants.kClimb;
                break;
        }
    }
    public ClimbState getClimbState(){
        return kState;
    }
    public boolean isAtSetpoint(){
        return kAtSetpoint;
    }
}