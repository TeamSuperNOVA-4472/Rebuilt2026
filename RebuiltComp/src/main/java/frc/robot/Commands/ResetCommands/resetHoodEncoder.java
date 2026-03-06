package frc.robot.Commands.ResetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;

public class resetHoodEncoder extends Command {
    private final FlywheelSubsystem kFlywheelSubsystem;

    public resetHoodEncoder(FlywheelSubsystem mFlywheelSubsystem)
    {
        kFlywheelSubsystem = mFlywheelSubsystem;

        addRequirements(kFlywheelSubsystem);
    }

    @Override
    public void initialize() {
        kFlywheelSubsystem.disableHoodPID();
        kFlywheelSubsystem.spinHood(FlywheelConstants.kResetHoodSpeed);
    }

    @Override
    public boolean isFinished() {
        return kFlywheelSubsystem.getStatorHood() > FlywheelConstants.kResetHoodStatorThreshold;
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
