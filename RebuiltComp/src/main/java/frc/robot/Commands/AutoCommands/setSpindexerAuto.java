package frc.robot.Commands.AutoCommands;

import java.lang.reflect.Constructor;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class setSpindexerAuto extends InstantCommand {
    private SpindexerSubsystem kSpindexer;
    private SpindexerMode kNewMode;
        
    public setSpindexerAuto(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;

        addRequirements(kSpindexer);
    }

    @Override
    public void initialize(){
        kSpindexer.setMode(kNewMode);
    }
}
