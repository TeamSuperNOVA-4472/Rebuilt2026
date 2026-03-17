package frc.robot.Commands;

import java.lang.reflect.Constructor;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class setSpindexer extends Command {
    private SpindexerSubsystem kSpindexer;
    private SpindexerMode kNewMode;
    
    public setSpindexer(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
    }

    @Override
    public void initialize() {
        kSpindexer.setMode(kNewMode);
    }
}
