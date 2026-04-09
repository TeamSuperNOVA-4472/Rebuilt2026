package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

    public enum SpindexerMode{
        OFF,
        LOAD,
        ANTIJAM
    }
    private TalonFX kSpindexerMotor;
    private TalonFX kKickerMotor;
    private SpindexerMode kMode;
    private final BangBangController kKickerBangBang;
    private final BangBangController kSpindexerBangBang;

    public SpindexerSubsystem(){
        kMode = SpindexerMode.OFF;
        kSpindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorPort,SpindexerConstants.kSpindexerCanbus);
        kKickerMotor = new TalonFX(SpindexerConstants.kKickerMotorPort, SpindexerConstants.kKickerCanbus);

        kKickerBangBang = new BangBangController();
        kSpindexerBangBang = new BangBangController();
       
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
        kSpindexerCurrentConfig.SupplyCurrentLowerLimit = SpindexerConstants.kSpindexerSupplyLimit;
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

    private double getKickerVelocity()
    {
        return kKickerMotor.getVelocity().getValueAsDouble()*SpindexerConstants.kKickerGearing;
    }

    private double getSpindexerVelocity()
    {
        return kSpindexerMotor.getVelocity().getValueAsDouble()*SpindexerConstants.kSpindexerGearing;
    }

    private void moveSpindexer(){
        switch (kMode){
        case OFF:
            kSpindexerMotor.setVoltage(0);
            kKickerMotor.setVoltage(0);
            break;

        case LOAD:
            kSpindexerMotor.setVoltage(MathUtil.clamp(SpindexerConstants.kSpindexerV*SpindexerConstants.kSpindexerSpeed + kSpindexerBangBang.calculate(getSpindexerVelocity(), SpindexerConstants.kSpindexerSpeed), -10, 10));
            kKickerMotor.setVoltage(MathUtil.clamp(SpindexerConstants.kKickerV*SpindexerConstants.kKickerSpeed + kKickerBangBang.calculate(getKickerVelocity(), SpindexerConstants.kKickerSpeed), -10, 10));
            //SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Kicker PID Output: ", output)
            break;
        case ANTIJAM:
            kSpindexerMotor.setVoltage(SpindexerConstants.kAntijamVoltage);
            kKickerMotor.setVoltage(SpindexerConstants.kAntijamVoltage);
            break;
        }
    }

    public SpindexerMode getMode(){
        return kMode;
    }
    public void setMode(SpindexerMode mNewMode){
        kMode = mNewMode;
        SmartDashboard.putString("Subsystems/SpindexerSubsystem/Spindexer Mode: ", kMode.name());
    }

    @Override
    public void periodic() {
        moveSpindexer();
        SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Spindexer Speed: ", getSpindexerVelocity());
        SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Kicker Speed: ", getKickerVelocity());

        SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Spindexer Supply Current: ", kSpindexerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/SpindexerSubsystem/Spindexer Stator Current: ", kSpindexerMotor.getStatorCurrent().getValueAsDouble());
    }
}