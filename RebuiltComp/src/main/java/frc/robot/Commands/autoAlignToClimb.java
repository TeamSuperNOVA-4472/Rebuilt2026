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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class autoAlignToClimb extends SequentialCommandGroup {
    private final SwerveSubsystem kSwerve;
    private final ClimbSubsystem kClimb;
    private final ClimbDirection kDirection;
    private final VisionSubsystem kVision;
    private Pose2d kTargetPose;

    public enum ClimbDirection {
        LEFT,
        RIGHT
    }

    private final ProfiledPIDController kGyroController = new ProfiledPIDController(
        0.5,
        0,
        0.001,
        new Constraints(
            720,
            360
        ));

    private final ProfiledPIDController kXController = new ProfiledPIDController(
        2,
        0,
        0,
        new Constraints(
            1.0,
            1.0
        ));

    private final ProfiledPIDController kYController = new ProfiledPIDController(
        2,
        0,
        0,
        new Constraints(
            1.0,
            1.0
        ));

    public autoAlignToClimb(SwerveSubsystem mSwerve, VisionSubsystem mVision, ClimbSubsystem mClimb, ClimbDirection mDirection)
    {
        kSwerve = mSwerve;
        kVision = mVision;
        kClimb = mClimb;
        kDirection = mDirection;

        kXController.setTolerance(0.2);
        kYController.setTolerance(0.2);
        kGyroController.setTolerance(0.5);

        kGyroController.enableContinuousInput(0, 360);

        switch (kDirection)
        {
            case LEFT:
                kTargetPose = FieldMathHelpers.isRedAlliance() ? ClimbConstants.kRedLeftClimbPose : ClimbConstants.kBlueLeftClimbPose;
                break;
            case RIGHT:
                kTargetPose = FieldMathHelpers.isRedAlliance() ? ClimbConstants.kRedRightClimbPose : ClimbConstants.kBlueRightClimbPose;
                break;
        }

        addCommands(
            //new InstantCommand(() -> kVision.restrictToClimbTags()),
            //goToPlace(new Pose2d(mSwerve.getPose().getX(), kTargetPose.getY(), kTargetPose.getRotation())).until(() -> isDrivetrainReady()),
            goToPlace(kTargetPose).until(() -> isDrivetrainReady())
            //new InstantCommand(() -> kVision.restrictToHubTags())
        );

        addRequirements(mSwerve, mClimb);
    }

    public boolean isDrivetrainReady() {
        return kXController.atGoal() && kYController.atGoal() && kGyroController.atGoal();
    }

    void driveSomewhere(Pose2d pose)
    {
        Pose2d currentPose = kSwerve.getPose();
        
        double xVelocity = -kXController.calculate(currentPose.getX(), pose.getX());
        double yVelocity = -kYController.calculate(currentPose.getY(), pose.getY());
        double rotVelocity = kGyroController.calculate(kSwerve.getHeadingDegrees(), pose.getRotation().getDegrees());

        //SmartDashboard.putString("Errors: ", kXController.getPositionError() + " " + kYController.getPositionError() + " " + kGyroController.getPositionError());
        ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, Units.degreesToRadians(rotVelocity));
        
        kSwerve.driveFieldOriented(speeds);

    }

    public Command goToPlace(Pose2d pose)
    {
        return new RunCommand(() -> driveSomewhere(pose)).beforeStarting(() -> {
            kXController.reset(kSwerve.getPose().getX());
            kYController.reset(kSwerve.getPose().getY());
            kGyroController.reset(kSwerve.getHeadingDegrees());
        }
        );
    }

    
}
