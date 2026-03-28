package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.ClimbSubsystem.ClimbState;

public class setClimb extends Command {
    private final ClimbSubsystem kClimb;
    private ClimbState kState;

    public setClimb(ClimbSubsystem mClimb, ClimbState mState){
        kClimb = mClimb;
        kState = mState;

        addRequirements(kClimb);
    }

    @Override
    public void initialize(){
        kClimb.setState(kState);
    }

    @Override
    public boolean isFinished(){
        return kClimb.isAtSetpoint();
    }

    @Override
    public void end(boolean isInterrupted){
        if (isInterrupted) kClimb.setState(ClimbState.STORED);
    }
}