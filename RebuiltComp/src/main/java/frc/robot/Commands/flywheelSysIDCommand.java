package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.FlywheelSubsystem;

public class flywheelSysIDCommand extends SequentialCommandGroup{
    public flywheelSysIDCommand(FlywheelSubsystem mFlywheelSubsystem){
        addRequirements(mFlywheelSubsystem);
        addCommands(
            mFlywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            mFlywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(10),
            mFlywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(10),
            mFlywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
    }

}
