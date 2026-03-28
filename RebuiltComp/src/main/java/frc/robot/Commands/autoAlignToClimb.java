package frc.robot.Commands;

import java.lang.module.ModuleDescriptor.Exports.Modifier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class autoAlignToClimb extends SequentialCommandGroup {
    SwerveSubsystem kSwerve;
    ClimbSubsystem kClimb;
    ClimbDirection kDirection;
    Pose2d[] kPoses = new Pose2d[2];

    public enum ClimbDirection {
        LEFT,
        RIGHT
    }

    HolonomicDriveController kController = new HolonomicDriveController(
        new PIDController(1.5,0,0),
        new PIDController(1.5, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

    public autoAlignToClimb(SwerveSubsystem mSwerve, ClimbSubsystem mClimb, ClimbDirection mDirection)
    {
        kSwerve = mSwerve;
        kClimb = mClimb;
        kDirection = mDirection;

        kController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);

        kController.setTolerance(new Pose2d(0.5, 0.5,Rotation2d.fromDegrees(5)));

        switch (kDirection)
        {
            case LEFT:
                kPoses[0] = new Pose2d(new Translation2d(15.52, 3.63), Rotation2d.fromDegrees(270));
                kPoses[1] = new Pose2d(new Translation2d(15.483078, 3.880612), Rotation2d.fromDegrees(270));
                break;
            case RIGHT:
                kPoses[0] = new Pose2d(new Translation2d(14.483078, 4.896612), Rotation2d.fromDegrees(90));
                kPoses[1] = new Pose2d(new Translation2d(15.483078, 4.896612), Rotation2d.fromDegrees(90));
                break;
        }

        addCommands(
            goToPose(kPoses[0]),
            goToPose(kPoses[1])
        );

        addRequirements(mSwerve, mClimb);
    }

    Command goToPose(Pose2d pose)
    {
        return new RunCommand(() -> driveSomewhere(pose)).until(kController::atReference);
    }

    void driveSomewhere(Pose2d pose)
    {
        ChassisSpeeds speeds = kController.calculate(kSwerve.getPose(), pose, 1.0, kSwerve.getPose().getRotation());
        SmartDashboard.putString("Speeds: ", speeds.toString());
        kSwerve.driveFieldOriented(speeds);
    }
}
