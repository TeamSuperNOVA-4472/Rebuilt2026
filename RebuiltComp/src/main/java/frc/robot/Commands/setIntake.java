package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeMode;

public class setIntake extends Command {
    private IntakeSubsystem kIntake;
    private IntakeSubsystem.IntakeMode kMode;
    public static int numRuns = 0;
    public setIntake(IntakeSubsystem mIntake, IntakeSubsystem.IntakeMode mMode){
        kIntake = mIntake;
        kMode = mMode;
        addRequirements(kIntake);
    }

    @Override
    public void initialize(){
        numRuns++;
        SmartDashboard.putNumber("numRuns", numRuns);
        if (kMode == kIntake.getMode()){ 
             kIntake.setIntake(IntakeMode.STORED);
        } else {
            kIntake.setIntake(kMode);
        }
    }

    @Override
    public boolean isFinished(){
        return kIntake.isReady();
    }
}
