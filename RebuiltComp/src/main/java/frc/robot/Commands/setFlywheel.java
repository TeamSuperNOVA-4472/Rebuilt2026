package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.FieldMathHelpers.Location;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kDistance;
    private final Supplier<Double> kVTowardHub;
    private final Supplier<FieldMathHelpers.Location> kGetLocation;

    public setFlywheel(
        FlywheelSubsystem mFlywheelSubsystem,
        Supplier<Double> mDistance,
        Supplier<Double> mVTowardHub,
        Supplier<FieldMathHelpers.Location> mGetLocation){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;
        kVTowardHub = mVTowardHub;
        kGetLocation = mGetLocation;

        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        double angle;
        double speed;
        double distance = kDistance.get();
        switch (kGetLocation.get()) {
            case TRENCH: // Hide hood under trench
                kFlywheel.setHoodTarget(FlywheelConstants.kHoodMinAngle);
                break;
            case ALLIANCE_ZONE: // Shoot to hub
                if (distance >= FlywheelConstants.kDistanceThresholdInMeters && distance <= FlywheelConstants.kDistanceMaximumInMeters){
                    speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
                    speed *= (1 - kVTowardHub.get() / speed);
                    angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
                    kFlywheel.setHoodTarget(angle);
                    kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
                }
                break;
            default: // Default to passing mode
            // TODO: make dynamic speeds
                kFlywheel.setHoodTarget(FlywheelConstants.kPassingAngle);
                kFlywheel.setMode(FlywheelMode.SPINNING, FlywheelConstants.kPassingSpeed);
        }
    }
}