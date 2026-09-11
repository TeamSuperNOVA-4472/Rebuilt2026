package frc.robot.Commands.ResetCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;

public class resetHoodEncoder extends Command {
    private final FlywheelSubsystem kFlywheelSubsystem;
    private final Debouncer kDebounce;

    public resetHoodEncoder(FlywheelSubsystem mFlywheelSubsystem)
    {
        kFlywheelSubsystem = mFlywheelSubsystem;
        kDebounce = new Debouncer(FlywheelConstants.kHoodDebounceTime);

        addRequirements(kFlywheelSubsystem);
    }

    @Override
    public void initialize() {
        kFlywheelSubsystem.disableHoodPID();
        kFlywheelSubsystem.spinHood(FlywheelConstants.kResetHoodSpeed);
        kDebounce.calculate(false);
    }

    @Override
    public boolean isFinished() {
        return kDebounce.calculate(kFlywheelSubsystem.getStatorHood() > FlywheelConstants.kResetHoodStatorThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        kFlywheelSubsystem.stopHood();
        if (!interrupted)
        {
            kFlywheelSubsystem.resetEncoderToBase();
            kFlywheelSubsystem.enableHoodPID();
        }
    }
}
