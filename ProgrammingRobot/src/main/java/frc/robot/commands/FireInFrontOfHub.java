package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class FireInFrontOfHub extends Command{
    private SwerveSubsystem kSwerve;
    private SpindexerSubsystem kSpindexer;
    private boolean isDone = false;
    public FireInFrontOfHub(SwerveSubsystem mSwerve, SpindexerSubsystem mSpindexer){
        kSwerve = mSwerve;
        kSpindexer = mSpindexer;
    }
    @Override
    public void execute(){
        if (kSwerve.getPose().getMeasureY().magnitude() > 3.9 && kSwerve.getPose().getMeasureY().magnitude() < 4.1){
            isDone = true;
            new ToggleSpindexer(kSpindexer);
        }
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
}
