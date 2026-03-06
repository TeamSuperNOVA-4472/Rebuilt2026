package frc.robot.Commands.SafeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelSafe extends InstantCommand {
    private final FlywheelSubsystem kFlywheelSubsystem;

    public setFlywheelSafe(FlywheelSubsystem mFlywheelSubsystem)
    {
        kFlywheelSubsystem = mFlywheelSubsystem;

        addRequirements(kFlywheelSubsystem);
    }

    @Override
    public void initialize() {
        kFlywheelSubsystem.setHoodTarget(FlywheelConstants.kSafeAngle);
        kFlywheelSubsystem.setMode(FlywheelMode.SPINNING, FlywheelConstants.kSafeSpeed);
    }
}
