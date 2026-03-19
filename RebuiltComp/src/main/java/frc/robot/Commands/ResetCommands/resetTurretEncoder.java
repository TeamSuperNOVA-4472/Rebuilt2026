package frc.robot.Commands.ResetCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.TurretSubsystem;

public class resetTurretEncoder extends Command {
    private final TurretSubsystem kTurretSubsystem;
    private final Debouncer kDebounce;

    public resetTurretEncoder (TurretSubsystem mTurretSubsystem)
    {
        kTurretSubsystem = mTurretSubsystem;
        kDebounce = new Debouncer(0.25, DebounceType.kRising);

        addRequirements(kTurretSubsystem);
    }

    @Override
    public void initialize() {
        kTurretSubsystem.disablePID();
        kTurretSubsystem.rotate(TurretConstants.kTurretResetSpeed);
        kDebounce.calculate(false);
    }

    @Override
    public boolean isFinished() {
        return kDebounce.calculate(kTurretSubsystem.getStator() > TurretConstants.kTurretResetStatorThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        kTurretSubsystem.stop();
        // if (!interrupted)
        // {
            kTurretSubsystem.resetEncoder();
            kTurretSubsystem.enablePID();
        // }
    }
}
