package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase 
{
    private final TalonFX turretMotor;

    private static final double revolutions = 2048;

    private static final double kP = 0.0;

    private static final double kI = 0.0;

    private static final double kD = 0.0;

    private final PIDController pidController;

    public TurretSubsystem() 
    {
        turretMotor = new TalonFX(24);

        pidController = new PIDController(kP, kI, kD);

        pidController.enableContinuousInput(0, 360);
    }
//
    public void rotate(double speed) 
    {
        turretMotor.set(speed);
    }

    public void rotateVoltage(double theVoltage) 
    {
        turretMotor.setVoltage(theVoltage);
    }

    public void stop() 
    {
        turretMotor.stopMotor();
    }

    public double getAngle() 
    {
        double encoderPosition = turretMotor.getPosition().getValueAsDouble();

        return (encoderPosition / revolutions) * 360;
    }

    public void goToAngle(double targetAngle) 
    {
        double currentAngle = getAngle();

        double output = pidController.calculate(currentAngle, targetAngle);

        turretMotor.set(output);
    }

    public void resetPosition()
    {
        turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder position: ", turretMotor.getRotorPosition().getValue().div(5).in(Degrees));
    }

    public double getPIDOutput(double currentAngle, double targetAngle) 
    {
        double difference = targetAngle - currentAngle;

        if (difference > 180)
        {
            difference -= 360;
        }

        else if (difference < -180)
        {
            difference += 360;
        }

        return pidController.calculate(currentAngle, targetAngle);
    }
    
}
