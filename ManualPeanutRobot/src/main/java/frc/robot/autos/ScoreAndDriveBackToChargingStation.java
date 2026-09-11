package frc.robot.autos;

import frc.robot.commands.GoStraight;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ScoreAndDriveBackToChargingStation extends SequentialCommandGroup{

    public ScoreAndDriveBackToChargingStation(SwerveDriveSubsystem s_Swerve) {
        addCommands(
            new GoStraight(s_Swerve,1.0, 1.21), //placeholder for auto score
            new GoStraight(s_Swerve, 1.0, -2.6)
        );
    }
}
