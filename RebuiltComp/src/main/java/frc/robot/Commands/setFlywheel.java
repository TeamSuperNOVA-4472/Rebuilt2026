package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
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
        if (kDistance.get() < 2.7)
        {
            speed = (distance * 22.964) - 2.4943;
            angle = 20;
        }
        else
        {
            speed = (distance * 12.24) + 6.6129;
            angle = 30;
        }
        kFlywheel.setHoodTarget(angle);
        kFlywheel.setMode(FlywheelMode.SPINNING, speed);
    }
}