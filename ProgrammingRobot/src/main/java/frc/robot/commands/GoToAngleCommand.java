package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

import java.lang.Math;
import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

public class GoToAngleCommand extends Command 
{
    /*private final TurretSubsystem turret;
    private final Supplier<Double> chassisHeading;
    private final Supplier<Double> chassisSpeed;

    private final double maxVoltage;

    private final double kDeadZoneStart = 330;
    private final double kDeadZoneEnd = 30;
    
    private double targetAngle;
    private double initialHeading;

    public GoToAngleCommand(TurretSubsystem turret, Supplier<Double> chassisHeading, Supplier<Double> chassisSpeed, double targetAngle, double maxVoltage) 
    {
        this.turret = turret;  
        this.chassisHeading = chassisHeading;
        this.chassisSpeed = chassisSpeed;
        this.targetAngle = targetAngle;
        this.maxVoltage = maxVoltage;

        initialHeading = chassisHeading.get();
        
        addRequirements(turret);
    }

    @Override
    public void execute() 
    {
        //double currentAngle = turret.getAngle(); // Gets turret angle
        double currentHeading = chassisHeading.get(); // Gets chassis angle

        targetAngle = (targetAngle - (currentHeading - initialHeading)) % 360;
        if (targetAngle < 0)
        {
            targetAngle = 360 + targetAngle;
        } // Calculate new setpoint

        SmartDashboard.putNumber("Target Angle: ", targetAngle);

        double feedAngle;
        if (targetAngle > kDeadZoneStart)
        {
            feedAngle = kDeadZoneStart;
        }
        else if (targetAngle < kDeadZoneEnd)
        {
            feedAngle = kDeadZoneEnd;
        }
        else
        {
            feedAngle = targetAngle;
        }

        SmartDashboard.putNumber("Feed Angle: ", feedAngle);

        double output = turret.getPIDOutput(currentAngle, feedAngle);
        SmartDashboard.putNumber("PID Output: ", output);

        turret.rotateVoltage(MathUtil.clamp(output, -maxVoltage, maxVoltage) + turret.getFeedForwardOutput(chassisSpeed.get()));

        initialHeading = currentHeading;
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.stop();
    }*/
}
