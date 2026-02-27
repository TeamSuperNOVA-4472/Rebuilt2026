package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeActionMode;

public class setIntakeAction extends Command {
    private IntakeSubsystem kIntake;
    private IntakeSubsystem.IntakeActionMode kActionMode;

    public setIntakeAction(IntakeSubsystem mIntake, IntakeSubsystem.IntakeActionMode mActionMode){
        kIntake = mIntake;
        kActionMode = mActionMode;
        addRequirements(kIntake);
    }

    @Override
    public void initialize(){
        kIntake.setIntakeAction(kActionMode);
    }
}
