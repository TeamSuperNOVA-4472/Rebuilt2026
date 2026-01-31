package frc.robot.Commands;

import java.lang.reflect.Constructor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class setspindexer extends InstantCommand {
    private SpindexerSubsystem kSpindexer;
    private SpindexerMode kNewMode;
    
    public setspindexer(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
    }
    @Override
    public void initialize(){
        kSpindexer.setMode(kNewMode);
    }
}