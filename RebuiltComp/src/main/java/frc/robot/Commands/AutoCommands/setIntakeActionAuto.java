package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.IntakeSubsystem;

public class setIntakeActionAuto extends InstantCommand {
    private IntakeSubsystem kIntake;
    private IntakeSubsystem.IntakeActionMode kActionMode;

    public setIntakeActionAuto(IntakeSubsystem mIntake, IntakeSubsystem.IntakeActionMode mActionMode){
        kIntake = mIntake;
        kActionMode = mActionMode;
    }

    @Override
    public void initialize(){
        kIntake.setIntakeAction(kActionMode);
    }
}
