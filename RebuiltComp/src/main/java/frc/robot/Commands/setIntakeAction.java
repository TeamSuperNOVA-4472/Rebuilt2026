package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.IntakeSubsystem;

public class setIntakeAction extends InstantCommand {
    private IntakeSubsystem kIntake;
    private IntakeSubsystem.IntakeActionMode kActionMode;

    public setIntakeAction(IntakeSubsystem mIntake, IntakeSubsystem.IntakeActionMode mActionMode){
        kIntake = mIntake;
        kActionMode = mActionMode;
    }

    @Override
    public void initialize(){
        kIntake.setIntakeAction(kActionMode);
    }
}
