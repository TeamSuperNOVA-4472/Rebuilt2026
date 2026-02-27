package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kAngle;

    public setFlywheel(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mAngle){
        kAngle = mAngle;
        kFlywheel = mFlywheelSubsystem;
        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        kFlywheel.setHoodTarget(kAngle.get());
        kFlywheel.setMode(FlywheelMode.SPINNING);
    }
}