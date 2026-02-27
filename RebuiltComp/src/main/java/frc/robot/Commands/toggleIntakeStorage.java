package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeActionMode;
import frc.robot.Subsystems.IntakeSubsystem.IntakeStorageMode;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class toggleIntakeStorage extends InstantCommand {
    private IntakeSubsystem kIntake;

    public toggleIntakeStorage(IntakeSubsystem mIntake){
        kIntake = mIntake;
        addRequirements(kIntake);
    }

    @Override
    public void initialize(){
        IntakeSubsystem.IntakeStorageMode storage;
        
        if (kIntake.getStorageMode().equals(IntakeStorageMode.OUT)){
            storage = IntakeStorageMode.STORED;
        }
        else{
            storage = IntakeStorageMode.OUT;
        }

        kIntake.setIntakeStorage(storage);
    }

}
