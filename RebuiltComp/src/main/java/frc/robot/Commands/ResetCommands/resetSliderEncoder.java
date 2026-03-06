package frc.robot.Commands.ResetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Subsystems.IntakeSubsystem;

public class resetSliderEncoder extends Command {
    private final IntakeSubsystem kIntakeSubsystem;

    public resetSliderEncoder(IntakeSubsystem mIntakeSubsystem)
    {
        kIntakeSubsystem = mIntakeSubsystem;

        addRequirements(kIntakeSubsystem);
    }

    @Override
    public void initialize() {
        kIntakeSubsystem.disablePID();
        kIntakeSubsystem.moveIntakeSlider(IntakeSubsystemConstants.kSliderResetSpeed);
    }

    @Override
    public boolean isFinished() {
        return kIntakeSubsystem.getSliderStator() > IntakeSubsystemConstants.kSliderResetStatorThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        kIntakeSubsystem.stopIntakeSlider();
        if (!interrupted)
        {
            kIntakeSubsystem.resetSliderEncoderToOutPosition();
            kIntakeSubsystem.enablePID();
        }
    }
}
