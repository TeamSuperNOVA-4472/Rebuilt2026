package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kDistance;

    public setFlywheel(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mDistance){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;
        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        double angle;
        double speed;
        double distance = kDistance.get();
        /* 
        if (kDistance.get() > 2.54)
        {
            speed = 47.5;
            angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
        }
        else
        {
            speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
            angle = 20;
        }*/
        SmartDashboard.putNumber("Distance To Hub: ", distance);
        //kFlywheel.setHoodTarget(angle);
        //kFlywheel.setMode(FlywheelMode.SPINNING, speed);
    }
}