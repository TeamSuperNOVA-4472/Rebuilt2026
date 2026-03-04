package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelTest extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Boolean> kSpeedUp;
    private final Supplier<Boolean> kSpeedDown;
    private final Supplier<Boolean> kHoodUp;
    private final Supplier<Boolean> kHoodDown;
    private double speed = 40;
    private double hood = 20;

    public setFlywheelTest(
        FlywheelSubsystem mFlywheelSubsystem, 
        Supplier<Boolean> mSpeedUp, 
        Supplier<Boolean> mSpeedDown,
        Supplier<Boolean> mHoodUp,
        Supplier<Boolean> mHoodDown){
        kFlywheel = mFlywheelSubsystem;
        kSpeedUp = mSpeedUp;
        kSpeedDown = mSpeedDown;
        kHoodUp = mHoodUp;
        kHoodDown = mHoodDown;

        addRequirements(kFlywheel);
    }

    @Override
    public void execute() {
        if (kSpeedUp.get())
        {
            speed += 2;
        }
        else if (kSpeedDown.get())
        {
            speed -= 2;
        }

        if (kHoodUp.get())
        {
            hood += 1;
        }
        else if (kHoodDown.get())
        {
            hood -= 1;
        }

        kFlywheel.setHoodTarget(hood);
        kFlywheel.setMode(FlywheelMode.SPINNING, speed);
    }
}
