package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kDistance;
    private final Supplier<Boolean> kIsScoring;

    public setFlywheel(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mDistance, Supplier<Boolean> mIsScoring){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;
        kIsScoring = mIsScoring;

        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        double angle;
        double speed;
        double distance = kDistance.get();
        if (kIsScoring.get())
        {
            if (distance >= FlywheelConstants.kDistanceThresholdInMeters && distance <= FlywheelConstants.kDistanceMaximumInMeters){
                speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
                angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
                kFlywheel.setHoodTarget(angle);
                kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
            }
        }
        else
        {
            kFlywheel.setHoodTarget(FlywheelConstants.kPassingAngle);
            kFlywheel.setMode(FlywheelMode.SPINNING, FlywheelConstants.kPassingSpeed);
        }
    }
}