package frc.robot.Commands.TeleopCommands;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldMathHelpers.Location;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretTeleop extends InstantCommand{
    private final TurretSubsystem kTurret;
    private final Supplier<Double> kGetHeadingDegrees;
    private final Supplier<Pose2d> kPose;
    private final Supplier<ChassisSpeeds> kSpeeds;
    private final Supplier<Double> kAngularVelocity;
    private final Supplier<Location> kLocation;

    public TurretTeleop(
        TurretSubsystem mTurret, 
        Supplier<Double> mGetHeadingDegrees, 
        Supplier<Pose2d> mPose,
        Supplier<ChassisSpeeds> mSpeeds,
        Supplier<Double> mAngularVelocity,
        Supplier<Location> mLocation)
    {
        kGetHeadingDegrees = mGetHeadingDegrees;
        kTurret = mTurret;
        kPose = mPose;
        kSpeeds = mSpeeds;
        kAngularVelocity = mAngularVelocity;
        kLocation = mLocation;

        addRequirements(kTurret);
    }

    @Override
    public void execute() {
        double kAbsTargetAngle;

        switch (kLocation.get())
        {
            case ALLIANCE_ZONE:
                ChassisSpeeds speeds = kSpeeds.get();
                kAbsTargetAngle = FieldMathHelpers.getRotationToHubWithSomeSpeed(kPose.get(), speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, kAngularVelocity.get());
                break;
            // TODO: make passing auto aim depending on field side
            default: kAbsTargetAngle = FieldMathHelpers.isRedAlliance() ? 0 : 180;
        }

        double kRelativeAngle = kGetHeadingDegrees.get() - kAbsTargetAngle + TurretConstants.kStartingAngleOffset;
        double kConstrainedAngle = (kRelativeAngle % 360 + 360) % 360;
        kTurret.setTargetAngle(kConstrainedAngle);

        SmartDashboard.putNumber("Subsystems/TurretSubsystem/Absolute Angle: ", kAbsTargetAngle);
    }

}
