package frc.robot.Commands;

import java.lang.reflect.Constructor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class setSpindexer extends InstantCommand {
    private SpindexerSubsystem kSpindexer;
    private SpindexerMode kNewMode;
    
    public setSpindexer(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
        addRequirements(kSpindexer);
    }

    @Override
    public void initialize(){
        kSpindexer.setMode(kNewMode);
    }
}
