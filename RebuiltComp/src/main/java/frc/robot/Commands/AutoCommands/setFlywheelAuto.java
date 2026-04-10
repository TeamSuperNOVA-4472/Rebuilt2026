package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelAuto extends InstantCommand {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kDistance;

    public setFlywheelAuto(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mDistance){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize(){
        double angle;
        double speed;
        double distance = kDistance.get();
            if (distance >= FlywheelConstants.kDistanceMinimumInMeters && distance <= FlywheelConstants.kDistanceMaximumInMeters){
                speed = FlywheelConstants.kDistanceToFlywheelSpeedDefault.get(distance);
                angle = FlywheelConstants.kDistanceToHoodAngleDefault.get(distance);
                kFlywheel.setHoodTarget(angle);
                kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
            }
    }
}