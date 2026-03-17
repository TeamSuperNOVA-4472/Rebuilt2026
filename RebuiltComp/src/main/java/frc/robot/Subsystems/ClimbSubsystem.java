package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX kClimbMotor;
    private ClimbMode kClimbMode = ClimbMode.STATIONARY;

    public enum ClimbMode{
        CLIMBING,
        STATIONARY
    }

    public ClimbSubsystem()
    {
        kClimbMotor = new TalonFX(ClimbConstants.kClimbMotorPort, ClimbConstants.kClimbCanbus);

        TalonFXConfiguration kClimbConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kClimbCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kClimbMotorConfig = new MotorOutputConfigs();
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

    
}