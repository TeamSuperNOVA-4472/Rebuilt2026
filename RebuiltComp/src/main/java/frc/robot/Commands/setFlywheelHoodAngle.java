package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;

public class setFlywheelHoodAngle extends Command {
    private FlywheelSubsystem kFlywheel;
    private double kAngle;

    public setFlywheelHoodAngle(FlywheelSubsystem mFlywheelSubsystem, double mAngle){
        kAngle = mAngle;
        kFlywheel = mFlywheelSubsystem;
        addRequirements(kFlywheel);
    }
    @Override
    public void initialize(){
        kFlywheel.setHoodTarget(kAngle);
    }
    @Override
    public boolean isFinished(){
        return kFlywheel.getHoodAtTartget();
    }
}