package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.FlywheelSubsystem;

public class ShooterSysIdCommand extends SequentialCommandGroup{
    public ShooterSysIdCommand(FlywheelSubsystem mFlywheelSubsystem){
        addCommands(
            mFlywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            mFlywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            mFlywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
            mFlywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );
    }

}
