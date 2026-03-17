package frc.robot.Commands.Autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Commands.setFlywheel;
import frc.robot.Commands.setSpindexer;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class ShootPreloadFromStandstill extends ParallelCommandGroup {

    public ShootPreloadFromStandstill(FlywheelSubsystem flyWheel, SwerveSubsystem swerve, SpindexerSubsystem spindexer) {

        /*addCommands(
            new setFlywheel(
                flyWheel, 
                () -> FieldMathHelpers.getDistanceToHubWithSomeSpeed(
                    swerve.getPose(), 
                    swerve.getFieldRelativeSpeeds().vxMetersPerSecond,
                    swerve.getFieldRelativeSpeeds().vyMetersPerSecond),
                () -> FieldMathHelpers.Location.ALLIANCE_ZONE),
            new SequentialCommandGroup(
                new WaitCommand(5.0),
                new setSpindexer(
                    spindexer,
                    SpindexerMode.LOAD,
                    () -> flyWheel.getMode() == FlywheelMode.SPINNING)
            )
        );*/
    }
}
