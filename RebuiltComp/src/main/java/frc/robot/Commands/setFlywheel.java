package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private FlywheelSubsystem kFlywheel;
    private FlywheelMode kFlyMode;

    public setFlywheel(FlywheelSubsystem mFlywheel, FlywheelMode mFlyMode){
        kFlywheel = mFlywheel;
        kFlyMode = mFlyMode;
    }

    @Override
    public void initialize(){
        kFlywheel.setMode(kFlyMode);
    }
    @Override
    public boolean isFinished(){
        return kFlyMode == FlywheelMode.OFF || kFlywheel.getSpinSpeed() > 0.95;
    }
}