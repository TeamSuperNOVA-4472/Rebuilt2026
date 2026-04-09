package frc.robot.Commands.ResetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class antijam extends Command {
    private final SpindexerSubsystem kSpindexer;
    private final FlywheelSubsystem kFlywheel;

    public antijam(SpindexerSubsystem mSpindexer, FlywheelSubsystem mFlywheel)
    {
        kSpindexer = mSpindexer;
        kFlywheel = mFlywheel;

        addRequirements(mSpindexer, mFlywheel);
    }

    @Override
    public void execute() {
        kSpindexer.setMode(SpindexerMode.ANTIJAM);
        kFlywheel.setMode(FlywheelMode.ANTIJAM);
    }
}
