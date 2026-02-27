package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kSpindexer = new SpindexerSubsystem();
     public static final double kSpindexerSpeed = -0.9;

    public enum SpindexerMode{
        OFF,
        LOAD
    }
    private TalonFX kSpindexerMotor;
    private TalonFX kKickerMotor;
    private SpindexerMode kMode;

    private SpindexerSubsystem(){
        kMode = SpindexerMode.OFF;
        kSpindexerMotor = new TalonFX(30,"rio");
        kKickerMotor = new TalonFX(51, "CANivore");

        TalonFXConfiguration kSpindexerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kSpindexerCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kSpindexerMotorConfig = new MotorOutputConfigs();
        kSpindexerMotor.getConfigurator().refresh(kSpindexerConfig);
        kSpindexerMotor.getConfigurator().refresh(kSpindexerCurrentConfig);
        kSpindexerMotor.getConfigurator().refresh(kSpindexerMotorConfig);
        kSpindexerCurrentConfig.SupplyCurrentLimit = 40;
        kSpindexerCurrentConfig.SupplyCurrentLimitEnable = true;
        kSpindexerCurrentConfig.StatorCurrentLimitEnable = true;
        kSpindexerCurrentConfig.StatorCurrentLimit = 40;
        kSpindexerMotorConfig.NeutralMode = NeutralModeValue.Brake;
        kSpindexerConfig.withCurrentLimits(kSpindexerCurrentConfig);
        kSpindexerConfig.withMotorOutput(kSpindexerMotorConfig);
        kSpindexerMotor.getConfigurator().apply(kSpindexerConfig);

        TalonFXConfiguration kKickerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kKickerCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kKickerMotorConfig = new MotorOutputConfigs();
        kKickerMotor.getConfigurator().refresh(kKickerConfig);
        kKickerMotor.getConfigurator().refresh(kKickerCurrentConfig);
        kKickerMotor.getConfigurator().refresh(kKickerMotorConfig);
        kKickerCurrentConfig.SupplyCurrentLimit = 40;
        kKickerCurrentConfig.SupplyCurrentLimitEnable = true;
        kKickerCurrentConfig.StatorCurrentLimitEnable = true;
        kKickerCurrentConfig.StatorCurrentLimit = 40;
        kKickerMotorConfig.NeutralMode = NeutralModeValue.Coast;
        kKickerConfig.withCurrentLimits(kKickerCurrentConfig);
        kKickerConfig.withMotorOutput(kKickerMotorConfig);
        kKickerMotor.getConfigurator().apply(kKickerConfig);
    }

    private void moveSpindexer(){
        switch (kMode){
        case OFF:
            kSpindexerMotor.set(0);
            kKickerMotor.set(0);
            break;

        case LOAD:
            kSpindexerMotor.set(kSpindexerSpeed);
            kKickerMotor.set(kSpindexerSpeed);
            break;
        }
    }
    public double getSpinSpeed(){
        return kSpindexerMotor.get();
    }

    public SpindexerMode getMode(){
        return kMode;
    }
    public void setMode(SpindexerMode mNewMode){
        kMode = mNewMode;
        SmartDashboard.putNumber("Spindexer Mode", kMode.ordinal());
        moveSpindexer();
    }
}