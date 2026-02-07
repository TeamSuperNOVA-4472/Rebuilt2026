package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
    private SysIdRoutine kRoutine;
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Radians.mutable(0);
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);



    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheelMotor = new SparkMax(33, MotorType.kBrushless);
        kFlywheelFeedforward = new SimpleMotorFeedforward(0.0001, 0.0075);
        kFlywheelFeedback = new PIDController(0.13,0, 0.001);
        kRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(kFlywheelMotor::setVoltage, log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheelMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(kFlywheelMotor.getEncoder().getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(kFlywheelMotor.getEncoder().getVelocity(), RotationsPerSecond));
              }, this));
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
        // kFlywheelMotor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheelMotor.getEncoder().getVelocity()/6000, kTargetSpeed) + kFlywheelFeedforward.calculate(kFlywheelMotor.getEncoder().getVelocity()), -11, 11));
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return kRoutine.dynamic(direction);
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return kRoutine.quasistatic(direction);
    }
}