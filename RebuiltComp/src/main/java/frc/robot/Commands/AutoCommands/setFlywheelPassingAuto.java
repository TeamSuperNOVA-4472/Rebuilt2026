package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelPassingAuto extends InstantCommand {
    private final FlywheelSubsystem kFlywheel;

    public setFlywheelPassingAuto(FlywheelSubsystem mFlywheelSubsystem){
        kFlywheel = mFlywheelSubsystem;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize(){
        kFlywheel.setHoodTarget(FlywheelConstants.kPassingAngle);
    }
}