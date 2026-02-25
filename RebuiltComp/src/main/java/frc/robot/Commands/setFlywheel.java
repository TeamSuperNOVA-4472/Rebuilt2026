package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheel extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Double> kAngle;
    private final Supplier<Boolean> kButton;

    public setFlywheel(FlywheelSubsystem mFlywheelSubsystem, Supplier<Double> mAngle, Supplier<Boolean> mButton){
        kAngle = mAngle;
        kFlywheel = mFlywheelSubsystem;
        kButton = mButton;
        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        kFlywheel.setHoodTarget(kAngle.get());

        if (kButton.get()){
            kFlywheel.setMode(FlywheelMode.SPINNING);
        } else {
            kFlywheel.setMode(FlywheelMode.OFF);
        }
    }
}