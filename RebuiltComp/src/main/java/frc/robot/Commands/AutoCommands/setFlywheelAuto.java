package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelAuto extends InstantCommand {
    private final FlywheelSubsystem kFlywheel;
    private final double kDistance;
    private final boolean kIsScoring;

    public setFlywheelAuto(FlywheelSubsystem mFlywheelSubsystem, double mDistance, boolean mIsScoring){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;
        kIsScoring = mIsScoring;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize(){
        double angle;
        double speed;
        double distance = kDistance;
        if (kIsScoring)
        {
            if (distance >= FlywheelConstants.kDistanceThresholdInMeters && distance <= FlywheelConstants.kDistanceMaximumInMeters){
                speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
                angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
                kFlywheel.setHoodTarget(angle);
                kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
            }
        }
    }
}