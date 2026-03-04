package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kSpindexer = new SpindexerSubsystem();

    public enum SpindexerMode{
        OFF,
        LOAD
    }
    private TalonFX kSpindexerMotor;
    private TalonFX kKickerMotor;
    private SpindexerMode kMode;

    private SpindexerSubsystem(){
        kMode = SpindexerMode.OFF;
        kSpindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorPort,SpindexerConstants.kSpindexerCanbus);
        kKickerMotor = new TalonFX(SpindexerConstants.kKickerMotorPort, SpindexerConstants.kKickerCanbus);

        TalonFXConfiguration kSpindexerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kSpindexerCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kSpindexerMotorConfig = new MotorOutputConfigs();
        kSpindexerMotor.getConfigurator().refresh(kSpindexerConfig);
        kSpindexerMotor.getConfigurator().refresh(kSpindexerCurrentConfig);
        kSpindexerMotor.getConfigurator().refresh(kSpindexerMotorConfig);
        kSpindexerCurrentConfig.SupplyCurrentLimit = SpindexerConstants.kSpindexerSupplyLimit;
        kSpindexerCurrentConfig.SupplyCurrentLimitEnable = SpindexerConstants.kSpindexerSupplyLimitEnabled;
        kSpindexerCurrentConfig.StatorCurrentLimitEnable = SpindexerConstants.kSpindexerStatorLimitEnabled;
        kSpindexerCurrentConfig.StatorCurrentLimit = SpindexerConstants.kSpindexerStatorLimit;
        kSpindexerMotorConfig.NeutralMode = SpindexerConstants.kSpindexerNeutralMode;
        kSpindexerConfig.withCurrentLimits(kSpindexerCurrentConfig);
        kSpindexerConfig.withMotorOutput(kSpindexerMotorConfig);
        kSpindexerMotor.getConfigurator().apply(kSpindexerConfig);

        TalonFXConfiguration kKickerConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kKickerCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kKickerMotorConfig = new MotorOutputConfigs();
        kKickerMotor.getConfigurator().refresh(kKickerConfig);
        kKickerMotor.getConfigurator().refresh(kKickerCurrentConfig);
        kKickerMotor.getConfigurator().refresh(kKickerMotorConfig);
        kKickerCurrentConfig.SupplyCurrentLimit = SpindexerConstants.kKickerSupplyLimit;
        kKickerCurrentConfig.SupplyCurrentLimitEnable = SpindexerConstants.kKickerSupplyLimitEnabled;
        kKickerCurrentConfig.StatorCurrentLimitEnable = SpindexerConstants.kKickerStatorLimitEnabled;
        kKickerCurrentConfig.StatorCurrentLimit = SpindexerConstants.kKickerStatorLimit;
        kKickerMotorConfig.NeutralMode = SpindexerConstants.kKickerNeutralMode;
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
            kSpindexerMotor.set(SpindexerConstants.kSpindexerSpeed);
            kKickerMotor.set(SpindexerConstants.kSpindexerSpeed);
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
        SmartDashboard.putString("Subsystems/SpindexerSubsystem/Spindexer Mode: ", kMode.name());
        moveSpindexer();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Spindexer Speed: ", getSpinSpeed());
    }
}