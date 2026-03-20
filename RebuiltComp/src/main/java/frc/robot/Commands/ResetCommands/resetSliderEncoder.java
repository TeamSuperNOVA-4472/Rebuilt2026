package frc.robot.Commands.ResetCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Subsystems.IntakeSubsystem;

public class resetSliderEncoder extends Command {
    private final IntakeSubsystem kIntakeSubsystem;
    private final Debouncer kDebounce;

    public resetSliderEncoder(IntakeSubsystem mIntakeSubsystem)
    {
        kIntakeSubsystem = mIntakeSubsystem;
        kDebounce = new Debouncer(0.25);

        addRequirements(kIntakeSubsystem);
    }

    @Override
    public void initialize() {
        kIntakeSubsystem.disablePID();
        kIntakeSubsystem.moveIntakeSlider(IntakeSubsystemConstants.kSliderResetSpeed);
        kDebounce.calculate(false);
    }

    @Override
    public boolean isFinished() {
        return kDebounce.calculate(kIntakeSubsystem.getSliderStator() > IntakeSubsystemConstants.kSliderResetStatorThreshold);
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
