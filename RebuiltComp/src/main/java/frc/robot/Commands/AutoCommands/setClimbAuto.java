package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.ClimbSubsystem.ClimbState;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setClimbAuto extends InstantCommand {
    private final ClimbSubsystem kClimb;
    private ClimbState kState;

    public setClimbAuto(ClimbSubsystem mClimb, ClimbState mState ){
        kClimb = mClimb;
        kState = mState;

        addRequirements(kClimb);
    }

    @Override
    public void initialize(){
        kClimb.setState(kState);
    }
}