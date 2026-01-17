package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

import java.lang.Math;
import java.util.function.Supplier;

public class GoToAngleCommand extends Command 
{
    private final TurretSubsystem turret;
    private final Supplier<Double> chassisHeading;

    private final double maxVoltage;
    
    private double targetAngle;
    private double initialHeading;

    public GoToAngleCommand(TurretSubsystem turret, Supplier<Double> chassisHeading, double targetAngle, double maxVoltage) 
    {
        this.turret = turret;  
        this.chassisHeading = chassisHeading;
        this.targetAngle = targetAngle;
        this.maxVoltage = maxVoltage;

        initialHeading = chassisHeading.get();
        
        addRequirements(turret);
    }

    @Override
    public void execute() 
    {
        double currentAngle = turret.getAngle(); // Gets turret angle
        double currentHeading = chassisHeading.get(); // Gets chassis angle

        targetAngle = (targetAngle - (currentHeading - initialHeading)) % 315; // Calculate new setpoint

        double output = turret.getPIDOutput(currentAngle, targetAngle);

        turret.rotateVoltage(MathUtil.clamp(output, -maxVoltage, maxVoltage));

        initialHeading = currentHeading;
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.stop();
    }
}
