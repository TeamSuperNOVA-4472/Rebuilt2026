package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

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
    private SingleJointedArmSim kFlywheelHoodSim;
    private DCMotor kFlywheelHoodSimMotor;
    private double kOutput;
    private Mechanism2d kSimSpace;
    private MechanismRoot2d kSimRoot;
    private MechanismLigament2d kSimDisp;
    private SysIdRoutine kRoutine;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Radians.mutable(0);
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
    
    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheel1Motor = new TalonFX(60, "CANivore");
        kFlywheel2Motor = new TalonFX(20, "CANivore");
        kFlywheelHoodMotor = new TalonFX(6);
        kHoodPidController = new PIDController(0.01,0,0);
        kFlywheel1Feedforward = new SimpleMotorFeedforward(0, 1.249, 0.70345);
        kFlywheel2Feedforward = new SimpleMotorFeedforward(0, 1.2435, 0.65849);
        kFlywheelFeedback = new PIDController(0,0, 0);
        kTargetAngle = 19;
        kFlywheelHoodSimMotor = DCMotor.getKrakenX44(1);
        kFlywheelHoodSim = new SingleJointedArmSim(kFlywheelHoodSimMotor, 58.824, 0.011, 0.2159, 19 * Math.PI / 180.0,  45 * Math.PI / 180.0, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(60, 60);
        kSimRoot = kSimSpace.getRoot("base", 30, 30);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Turret",10 , kFlywheelHoodSim.getAngleRads() * 180 / Math.PI));

        kRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::setFlywheelVoltage, log -> {
                // Record a frame for the shooter motor.
                log.motor("FlywheelMotor1")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheel1Motor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(kFlywheel1Motor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(kFlywheel1Motor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                log.motor("FlywheelMotor2")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheel2Motor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(kFlywheel2Motor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(kFlywheel2Motor.getVelocity().getValueAsDouble(), RotationsPerSecond));
              }, this));
        SmartDashboard.putData("FlyWheelHoodSim", kSimSpace);
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
    public void setFlywheelVoltage(Voltage vIn){
        kFlywheel1Motor.setVoltage(vIn.magnitude());
        kFlywheel2Motor.setVoltage(vIn.magnitude());
    }
    public void setFlywheelVoltageDouble(double vIn){
        kFlywheel1Motor.setVoltage(vIn);
        kFlywheel2Motor.setVoltage(vIn);
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
        /*kHoodAtTarget = kHoodPidController.atSetpoint();
        if (Robot.isReal())kOutput = MathUtil.clamp(kHoodPidController.calculate(kFlywheelHoodMotor.getPosition().getValueAsDouble(),kTargetAngle), -1 , 1);
        else kOutput = MathUtil.clamp(kHoodPidController.calculate(kFlywheelHoodSim.getAngleRads() * 180 / Math.PI,kTargetAngle), -1 , 1);
        kFlywheelHoodMotor.set(kOutput);
        kFlywheel1Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel1Motor.getVelocity().getValueAsDouble()/512.0, kTargetSpeed) + kFlywheel1Feedforward.calculate(kFlywheel1Motor.getVelocity().getValueAsDouble()), -11, 11));
        kFlywheel2Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel2Motor.getVelocity().getValueAsDouble()/512.0, kTargetSpeed) + kFlywheel2Feedforward.calculate(kFlywheel2Motor.getVelocity().getValueAsDouble()), -11, 11));*/
    }
    @Override
    public void simulationPeriodic() {
      kFlywheelHoodSim.setInput(kOutput * 12.0);

      kFlywheelHoodSim.update(0.02);

      kSimDisp.setAngle(kFlywheelHoodSim.getAngleRads()*180 / Math.PI);
      SmartDashboard.putNumber("FlywheelHood", kFlywheelHoodSim.getAngleRads()*180 / Math.PI);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return kRoutine.dynamic(direction);
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return kRoutine.quasistatic(direction);
    }
}