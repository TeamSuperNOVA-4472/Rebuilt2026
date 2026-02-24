package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelMode;

public class FlywheelTestCommand extends ParallelCommandGroup{
    private SwerveSubsystem kSwerve;
    private FlywheelSubsystem kFlywheel;
    private SpindexerSubsystem kSpindexer;
    /* public FlywheelTestCommand(SwerveSubsystem mSwerve, FlywheelSubsystem mFlywheel, SpindexerSubsystem mSpindexer){
        kSwerve = mSwerve;
        kFlywheel = mFlywheel;
        kSpindexer = mSpindexer;
        addCommands(
            new PathPlannerAuto("ShooterTester"),
            new calculateFlywheelSpeed(mFlywheel, mSwerve, FlywheelMode.SPINNING, FlywheelConstants.kDistanceToVelocity.get(2.6)),
            new FireInFrontOfHub(mSwerve, mSpindexer)
        );
    } */
}
