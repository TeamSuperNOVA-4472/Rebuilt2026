package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase 
{
    private final TalonFX kTurretMotor;

    private static final double kRevolutions = 2048;

    private static final double kP = 0.0;

    private static final double kI = 0.0;

    private static final double kD = 0.0;

    private final PIDController kPidController;

    public TurretSubsystem() 
    {
        kTurretMotor = new TalonFX(24);

        kPidController = new PIDController(kP, kI, kD);
    }

    public void rotate(double speed) 
    {
        kTurretMotor.set(speed);
    }

    public void stop() 
    {
        kTurretMotor.stopMotor();
    }

    public double getAngle() 
    {
        double encoderPosition = kTurretMotor.getPosition().getValueAsDouble();

        return (encoderPosition / kRevolutions) * 360;
    }

    public void goToAngle(double targetAngle) 
    {
        double currentAngle = getAngle();

        double output = kPidController.calculate(currentAngle, targetAngle);

        kTurretMotor.set(output);
    }
}
