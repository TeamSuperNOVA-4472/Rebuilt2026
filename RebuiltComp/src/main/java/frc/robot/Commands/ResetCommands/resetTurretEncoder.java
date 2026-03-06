package frc.robot.Commands.ResetCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.TurretSubsystem;

public class resetTurretEncoder extends Command {
    private final TurretSubsystem kTurretSubsystem;
    private final Debouncer kDebouce;

    public resetTurretEncoder (TurretSubsystem mTurretSubsystem)
    {
        kTurretSubsystem = mTurretSubsystem;
        kDebouce = new Debouncer(1);

        addRequirements(kTurretSubsystem);
    }

    @Override
    public void initialize() {
        kTurretSubsystem.disablePID();
        kTurretSubsystem.rotate(TurretConstants.kTurretResetSpeed);
    }

    @Override
    public boolean isFinished() {
        return kDebouce.calculate(kTurretSubsystem.getStator() > TurretConstants.kTurretResetStatorThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        kTurretSubsystem.stop();
        if (!interrupted)
        {
            kTurretSubsystem.resetEncoder();
            kTurretSubsystem.enablePID();
        }
    }
}
