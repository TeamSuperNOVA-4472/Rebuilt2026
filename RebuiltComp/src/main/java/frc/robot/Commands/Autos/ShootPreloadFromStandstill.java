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

    private final FlywheelSubsystem flyWheel = FlywheelSubsystem.kFlywheel;
    private final SwerveSubsystem swerve = SwerveSubsystem.kSwerve;
    private final SpindexerSubsystem spindexer = SpindexerSubsystem.kSpindexer;

    public ShootPreloadFromStandstill() {

        addCommands(
            new setFlywheel(
                flyWheel, 
                () -> FieldMathHelpers.getDistanceToHubWithSomeSpeed(
                    swerve.getPose(), 
                    swerve.getFieldRelativeSpeeds().vxMetersPerSecond,
                    swerve.getFieldRelativeSpeeds().vyMetersPerSecond)),
            new SequentialCommandGroup(
                new WaitCommand(5.0),
                new ConditionalCommand(
                    new setSpindexer(spindexer, SpindexerMode.LOAD),
                    new InstantCommand(),
                    () -> flyWheel.getMode() == FlywheelMode.SPINNING)
            )
        );
    }
}
