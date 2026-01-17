package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class GoToAngleCommand extends Command 
{
    private final TurretSubsystem turret;

    private final double targetAngle;

    private final double maxVoltage;

    public GoToAngleCommand(TurretSubsystem turret, double targetAngle, double maxVoltage) 
    {
        this.turret = turret;

        this.targetAngle = targetAngle;

        this.maxVoltage = maxVoltage;
        
        addRequirements(turret);
    }

    @Override
    public void execute() 
    {
        double currentAngle = turret.getAngle();

        double difference = targetAngle - currentAngle;

        if (difference > 180) 
        {
            difference -= 360;
        } 
        
        else if (difference < -180) 
        {
            difference += 360;
        }

        double output = turret.getPIDOutput(currentAngle, targetAngle);

        turret.rotateVoltage(output);
    }

    @Override
    public boolean isFinished() 
    {
        return Math.abs(targetAngle - turret.getAngle()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.stop();
    }
}
