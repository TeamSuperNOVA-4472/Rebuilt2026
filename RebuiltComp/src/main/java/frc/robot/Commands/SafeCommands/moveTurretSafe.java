package frc.robot.Commands.SafeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.TurretSubsystem;

public class moveTurretSafe extends Command {
    private final TurretSubsystem kTurretSubsystem;

    public moveTurretSafe(TurretSubsystem mTurretSubsystem)
    {
        kTurretSubsystem = mTurretSubsystem;

        addRequirements(kTurretSubsystem);
    }

    @Override
    public void execute() {
        kTurretSubsystem.setTargetAngle(TurretConstants.kStartingAngleOffset);
    }
}
