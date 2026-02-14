package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem.SpindexerMode;

public class ToggleSpindexer extends InstantCommand {
    private SpindexerSubsystem kSpindexer;

    public ToggleSpindexer(SpindexerSubsystem mSpindexer){
        kSpindexer = mSpindexer;
    }
    @Override
    public void initialize(){
        if (kSpindexer.getMode() == SpindexerMode.LOAD){
            kSpindexer.setMode(SpindexerMode.OFF);
        } else {
            kSpindexer.setMode(SpindexerMode.LOAD);
        }
    }
}
