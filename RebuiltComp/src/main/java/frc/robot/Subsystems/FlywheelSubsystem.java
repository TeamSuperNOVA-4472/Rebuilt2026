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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FlywheelConstants;

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
    private boolean kFlywheelAtTarget;
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
        kFlywheel1Motor = new TalonFX(FlywheelConstants.kFlywheel1MotorPort, FlywheelConstants.kFlywheel1Canbus);
        kFlywheel2Motor = new TalonFX(FlywheelConstants.kFlywheel2MotorPort, FlywheelConstants.kFlywheel2Canbus);
        kFlywheelHoodMotor = new TalonFX(FlywheelConstants.kFlywheelHoodMotorPort, FlywheelConstants.kFlywheelHoodCanbus);

        kHoodPidController = new PIDController(FlywheelConstants.kPHood, FlywheelConstants.kIHood, FlywheelConstants.kDHood);
        kHoodPidController.setTolerance(FlywheelConstants.kHoodTolerance);
        kFlywheel1Feedforward = new SimpleMotorFeedforward(FlywheelConstants.kSFlywheel, FlywheelConstants.kVFlywheel, FlywheelConstants.kAFlywheel);
        kFlywheel2Feedforward = new SimpleMotorFeedforward(FlywheelConstants.kSFlywheel, FlywheelConstants.kVFlywheel, FlywheelConstants.kAFlywheel);
        kFlywheelFeedback = new PIDController(FlywheelConstants.kPFlywheel,FlywheelConstants.kIFlywheel, FlywheelConstants.kDFlywheel);
        kFlywheelFeedback.setTolerance(FlywheelConstants.kFlywheelTolerance);

        kTargetAngle = FlywheelConstants.kStartingHoodAngle;

        kFlywheelHoodSimMotor = DCMotor.getKrakenX44(FlywheelConstants.kSimNumMotors);
        kFlywheelHoodSim = new SingleJointedArmSim(kFlywheelHoodSimMotor, FlywheelConstants.kSimGearing, FlywheelConstants.kSimjKgMetersSquared, FlywheelConstants.kSimArmLength, Constants.FlywheelConstants.kHoodMinAngle * Math.PI / 180.0,  Constants.FlywheelConstants.kHoodMaxAngle * Math.PI / 180.0, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(FlywheelConstants.kSimWidth, FlywheelConstants.kSimHeight);
        kSimRoot = kSimSpace.getRoot(FlywheelConstants.kSimRootName, FlywheelConstants.kSimX, FlywheelConstants.kSimY);
        kSimDisp = kSimRoot.append(new MechanismLigament2d(FlywheelConstants.kSimName, FlywheelConstants.kSimLength, kFlywheelHoodSim.getAngleRads() * 180 / Math.PI));

        kRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::setFlywheelVoltage, log -> {
                // Record a frame for the shooter motor.
                log.motor("FlywheelMotor1")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheel1Motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(kFlywheel1Motor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(kFlywheel1Motor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                log.motor("FlywheelMotor2")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheel2Motor.get() * RobotController.getBatteryVoltage(), Volts))
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
        kFlywheel1CurrentConfig.SupplyCurrentLimit = FlywheelConstants.kFlywheel1SupplyLimit;
        kFlywheel1CurrentConfig.SupplyCurrentLimitEnable = FlywheelConstants.kFlywheel1SupplyLimitEnabled;
        kFlywheel1CurrentConfig.StatorCurrentLimitEnable = FlywheelConstants.kFlywheel1StatorLimitEnabled;
        kFlywheel1CurrentConfig.StatorCurrentLimit = FlywheelConstants.kFlywheel1StatorLimit;
        kFlywheel1MotorConfig.NeutralMode = FlywheelConstants.kFlywheel1NeutralMode;
        kFlywheel1Config.withCurrentLimits(kFlywheel1CurrentConfig);
        kFlywheel1Config.withMotorOutput(kFlywheel1MotorConfig);
        kFlywheel1Motor.getConfigurator().apply(kFlywheel1Config);

        TalonFXConfiguration kFlywheel2Config = new TalonFXConfiguration();
        CurrentLimitsConfigs kFlywheel2CurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kFlywheel2MotorConfig = new MotorOutputConfigs();
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2Config);
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2CurrentConfig);
        kFlywheel2Motor.getConfigurator().refresh(kFlywheel2MotorConfig);
        kFlywheel2CurrentConfig.SupplyCurrentLimit = FlywheelConstants.kFlywheel2SupplyLimit;
        kFlywheel2CurrentConfig.SupplyCurrentLimitEnable = FlywheelConstants.kFlywheel2SupplyLimitEnabled;
        kFlywheel2CurrentConfig.StatorCurrentLimitEnable = FlywheelConstants.kFlywheel2StatorLimitEnabled;
        kFlywheel2CurrentConfig.StatorCurrentLimit = FlywheelConstants.kFlywheel2StatorLimit;
        kFlywheel2MotorConfig.NeutralMode = FlywheelConstants.kFlywheel2NeutralMode;
        kFlywheel2Config.withCurrentLimits(kFlywheel2CurrentConfig);
        kFlywheel2Config.withMotorOutput(kFlywheel2MotorConfig);
        kFlywheel2Motor.getConfigurator().apply(kFlywheel2Config);

        // Config for the Hood motor 
        TalonFXConfiguration kFlywheelHoodConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kFlywheelHoodCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kFlywheelHoodMotorConfig = new MotorOutputConfigs();
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodConfig);
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodCurrentConfig);
        kFlywheelHoodMotor.getConfigurator().refresh(kFlywheelHoodMotorConfig);
        
        // Set current supply and stator limits for the hood motor
        kFlywheelHoodCurrentConfig.SupplyCurrentLimit = FlywheelConstants.kFlywheelHoodSupplyLimit;
        kFlywheelHoodCurrentConfig.SupplyCurrentLimitEnable = FlywheelConstants.kFlywheelHoodSupplyLimitEnabled;
        kFlywheelHoodCurrentConfig.StatorCurrentLimitEnable = FlywheelConstants.kFlywheelHoodStatorLimitEnabled;
        kFlywheelHoodCurrentConfig.StatorCurrentLimit = FlywheelConstants.kFlywheelHoodStatorLimit;
        kFlywheelHoodMotorConfig.NeutralMode = FlywheelConstants.kFlywheelHoodNeutralMode; // Set mode to braking
        kFlywheelHoodConfig.withCurrentLimits(kFlywheelHoodCurrentConfig);
        kFlywheelHoodConfig.withMotorOutput(kFlywheelHoodMotorConfig);
        kFlywheelHoodMotor.getConfigurator().apply(kFlywheelHoodConfig);

        kFlywheelHoodMotor.setPosition(0);
    }
    private void moveFlywheel(double speed){
        switch (kMode) {
        case OFF:
            kTargetSpeed = 0;
            break;
        case SPINNING:
            kTargetSpeed = speed;
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
    public void setMode(FlywheelMode mNewMode, double speed){
        kMode = mNewMode;
        moveFlywheel(speed);
    }
    public void setHoodTarget(double mNewTarget){
        if (mNewTarget >= Constants.FlywheelConstants.kHoodMinAngle && mNewTarget <= Constants.FlywheelConstants.kHoodMaxAngle) kTargetAngle = mNewTarget;
    }

    public double getHoodAngle()
    {
        return kFlywheelHoodMotor.getPosition().getValueAsDouble() * FlywheelConstants.kHoodEncoderMultiplier + FlywheelConstants.kHoodMinAngle;
    }
 
    public boolean getHoodAtTarget(){
        return kHoodAtTarget;
    }

    public boolean getFlywheelAtTarget(){
        return kFlywheelAtTarget;
    }

    @Override
    public void periodic(){
        kHoodAtTarget = kHoodPidController.atSetpoint();
        kFlywheelAtTarget = kFlywheelFeedback.atSetpoint();
        if (Robot.isReal()) kOutput = MathUtil.clamp(kHoodPidController.calculate(kFlywheelHoodMotor.getPosition().getValueAsDouble() * Constants.FlywheelConstants.kHoodEncoderMultiplier + Constants.FlywheelConstants.kHoodMinAngle,kTargetAngle), -FlywheelConstants.kMaxSpeed, FlywheelConstants.kMaxSpeed);
        else kOutput = MathUtil.clamp(kHoodPidController.calculate(kFlywheelHoodSim.getAngleRads() * 180 / Math.PI,kTargetAngle), -FlywheelConstants.kMaxSpeed , FlywheelConstants.kMaxSpeed);
        kFlywheelHoodMotor.set(kOutput);
        kFlywheel1Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel1Motor.getVelocity().getValueAsDouble(), kTargetSpeed) + kFlywheel1Feedforward.calculate(kTargetSpeed), -FlywheelConstants.kMaxVoltage, FlywheelConstants.kMaxVoltage));
        kFlywheel2Motor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheel2Motor.getVelocity().getValueAsDouble(), kTargetSpeed) + kFlywheel2Feedforward.calculate(kTargetSpeed), -FlywheelConstants.kMaxVoltage, FlywheelConstants.kMaxVoltage));
        
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Actual Flywheel Speed 1: ", kFlywheel1Motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Actual Flywheel Speed 2: ", kFlywheel2Motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Actual Flywheel Voltage 1: ", kFlywheel1Motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Actual Flywheel Voltage 2: ", kFlywheel2Motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Target Flywheel Speed: ", kTargetSpeed);
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Current Hood Angle: ", getHoodAngle());
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Target Hood Angle: ", kTargetAngle);
        SmartDashboard.putNumber("Subsystems/FlywheelSubsystem/Hood PID Output: ", kOutput);
    }
    @Override
    public void simulationPeriodic() {
      kFlywheelHoodSim.setInput(kOutput * FlywheelConstants.kSimMultiplier);

      kFlywheelHoodSim.update(FlywheelConstants.kSimdt);

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