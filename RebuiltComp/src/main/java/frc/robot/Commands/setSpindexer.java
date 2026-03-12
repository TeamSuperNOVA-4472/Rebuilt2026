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
    private Supplier<Boolean> kReadyToShoot;
    
    public setSpindexer(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode, Supplier<Boolean> mReadyToShoot){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
        kReadyToShoot = mReadyToShoot;

        addRequirements(kSpindexer);
    }

    @Override
    public void execute(){
        kSpindexer.setMode(kNewMode);
    }
}
