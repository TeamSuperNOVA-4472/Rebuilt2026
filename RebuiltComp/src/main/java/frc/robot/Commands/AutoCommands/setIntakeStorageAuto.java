package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.IntakeSubsystem;

public class setIntakeStorageAuto extends InstantCommand {
    private IntakeSubsystem kIntake;
    private IntakeSubsystem.IntakeStorageMode kStore;

    public setIntakeStorageAuto(IntakeSubsystem mIntake, IntakeSubsystem.IntakeStorageMode mStore){
        kIntake = mIntake;
        kStore = mStore;
    }

    @Override
    public void initialize(){
        kIntake.setIntakeStorage(kStore);
    }

}
