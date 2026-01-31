package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase 
{
    private final TalonFX mTurretMotor;
    private final TalonFXSimState mTurretMotorSim;

    private final DCMotorSim mMotorSimModel;

    private static final double kThreeSixtyDegrees = 360;
    private static final double kSimRefreshTimeMS = 0.020;

    private static final double kP = 0.001; //0.0135
    private static final double kI = 0.0; //0.001
    private static final double kD = 0.0;

    private static final double kS = 0.0; //0.001
    private static final double kV = 0;

    private static final double kGearRatio = 5.0;

    private static final double kDeadZoneStart = 330;
    private static final double kOffset = 0;

    private final PIDController mPidController;
    private final SimpleMotorFeedforward mFeedForward;

    public TurretSubsystem() 
    {
        mTurretMotor = new TalonFX(24);
        mTurretMotorSim = mTurretMotor.getSimState();

        mMotorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.001, kGearRatio), 
                DCMotor.getKrakenX60(1)
                );

        mPidController = new PIDController(kP, kI, kD);
        mFeedForward = new SimpleMotorFeedforward(kS, kV);
    }

    private double normalizeHeadingTo360Degrees(Angle angle)
    {
        return (angle.in(Degrees) % kThreeSixtyDegrees + kThreeSixtyDegrees) % kThreeSixtyDegrees;
    }

    private Angle getCurrentHeading()
    {
        return mTurretMotor.getRotorPosition().getValue();
    }

    public void rotateVoltage(double voltage) 
    {
        mTurretMotor.setVoltage(voltage);
    }

    private double adjustSetPoint(double setPoint)
    {
        if (setPoint >= kDeadZoneStart)
        {
            // Check if setpoint is closer to starting or ending bound of deadzone
            return setPoint > (kThreeSixtyDegrees + kDeadZoneStart) / 2 ? 0 : kDeadZoneStart;
        }
        return setPoint;
    }

    private double absoluteHeadingToRelativeTurretHeading(Angle desiredAbsoluteHeading, Angle drivetrainHeading)
    {
        return normalizeHeadingTo360Degrees(desiredAbsoluteHeading.minus(drivetrainHeading).plus(Degrees.of(kOffset)));
    }   

    public void turnToSetpoint(Angle drivetrainHeading, Angle desiredHeading, double desiredVelocity)
    {
        double dtHeading = (drivetrainHeading.in(Degrees) + 360) % 360;
        double newSetpoint = adjustSetPoint(absoluteHeadingToRelativeTurretHeading(desiredHeading, Degrees.of(dtHeading)));

        double feedBack = mPidController.calculate(getCurrentHeading().in(Degrees), newSetpoint);
        double feedForward = mFeedForward.calculate(desiredVelocity);

        rotateVoltage(feedBack + feedForward);

        SmartDashboard.putNumber("Drivetrain Heading: ", dtHeading);
        SmartDashboard.putNumber("Turret Heading: ", getCurrentHeading().in(Degrees));
        SmartDashboard.putNumber("Relative Heading: ", newSetpoint);

    }

    public void stop() 
    {
        mTurretMotor.stopMotor();
    }

    public void resetPosition()
    {
        mTurretMotor.setPosition(0);
    }

    @Override
    public void simulationPeriodic() {
        mTurretMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        Voltage motorVoltage = mTurretMotorSim.getMotorVoltageMeasure();

        mMotorSimModel.setInputVoltage(motorVoltage.in(Volts));
        mMotorSimModel.update(kSimRefreshTimeMS);

        mTurretMotorSim.setRawRotorPosition(mMotorSimModel.getAngularPosition().in(Rotations));
        mTurretMotorSim.setRotorVelocity(mMotorSimModel.getAngularVelocity().in(RotationsPerSecond));
    }

}
