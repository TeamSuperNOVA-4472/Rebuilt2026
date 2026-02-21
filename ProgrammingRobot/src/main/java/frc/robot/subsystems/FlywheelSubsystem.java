package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

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
    private final MutAngularVelocity kFlywheelSpeed = RotationsPerSecond.mutable(0);


    private FlywheelSubsystem(){
        kMode = FlywheelMode.OFF;
        //TODO: Values below should be constants.
        kFlywheelMotor = new SparkMax(33, MotorType.kBrushless);
        kFlywheelFeedforward = new SimpleMotorFeedforward(0.16693, 0.12451, 0.040519);
        //TODO: Tune the PID.
        kFlywheelFeedback = new PIDController(0.05,0, 0);
        kRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(kFlywheelMotor::setVoltage, log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            kFlywheelMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(kFlywheelMotor.getEncoder().getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(kFlywheelMotor.getEncoder().getVelocity()/60.0, RotationsPerSecond));
              }, this));
    }

    public double getTangentialSpeed()
    {
       return getTargetSpeed() * ((3.14*2*0.0508));    
    }

    public double getSpinSpeed(){
        return kFlywheelMotor.get();
    }
    public FlywheelMode getMode(){
        return kMode;
    }
    public void setTargetSpeed(double mNewSpeed){
        kTargetSpeed = mNewSpeed;
    }
    public double getTargetSpeed(){
        return kTargetSpeed;
    }
    public void setMode(FlywheelMode mNewMode){
        kMode = mNewMode;
    }
    @Override
    public void periodic(){
        kFlywheelSpeed.mut_replace(kFlywheelMotor.getEncoder().getVelocity()/60.0, RotationsPerSecond);
        SmartDashboard.putNumber("FlywheelPID Out", MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheelSpeed.magnitude(), kTargetSpeed) + kFlywheelFeedforward.calculate(kTargetSpeed), -11, 11));
        if (kMode == FlywheelMode.SPINNING) kFlywheelMotor.setVoltage(MathUtil.clamp(kFlywheelFeedback.calculate(kFlywheelSpeed.magnitude(), kTargetSpeed) + kFlywheelFeedforward.calculate(kTargetSpeed), -11, 11));
        else kFlywheelMotor.setVoltage(0);
        SmartDashboard.putNumber("Actual Speed", kFlywheelSpeed.magnitude());
        SmartDashboard.putNumber("Flywheel Feed Forward", kFlywheelFeedforward.calculate(kTargetSpeed));
        SmartDashboard.putNumber("Target Speed", kTargetSpeed);
        SmartDashboard.putNumber("Tangential Speed: ", getTangentialSpeed());
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return kRoutine.dynamic(direction);
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return kRoutine.quasistatic(direction);
    }
}