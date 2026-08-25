package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(SwerveDriveSubsystem pDrive, VisionSubsystem pVision) {
        addCommands(
            new LimeLightTurn(pDrive, pVision, 10),
            new GoStraight(pDrive, 1.0, 0.5));
    }
}
