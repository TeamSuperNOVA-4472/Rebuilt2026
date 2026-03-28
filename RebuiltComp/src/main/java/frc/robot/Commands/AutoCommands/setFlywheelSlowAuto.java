package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelSlowAuto extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kDistance;

    public setFlywheelSlowAuto(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mDistance){
        kFlywheel = mFlywheelSubsystem;
        kDistance = mDistance;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize(){
        double angle;
        double speed;
        double distance = kDistance.get();
        kFlywheel.disableFlywheelPID();
        SmartDashboard.putNumber("Distance setFlywheel", distance);
            if (distance >= FlywheelConstants.kDistanceMinimumInMeters && distance <= FlywheelConstants.kDistanceMaximumInMeters){
                speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
                angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
                kFlywheel.setHoodTarget(FlywheelConstants.kHoodMinAngle);
                kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
            }
    }

    @Override
    public boolean isFinished(){
        return kFlywheel.getFlywheelAtTarget();
    }

    @Override
    public void end(boolean isInterupted){
        kFlywheel.enableFlywheelPID();
    }
}
