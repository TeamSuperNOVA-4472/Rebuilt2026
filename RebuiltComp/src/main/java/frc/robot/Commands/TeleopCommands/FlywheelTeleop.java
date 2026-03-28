package frc.robot.Commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldMathHelpers;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.FieldMathHelpers.Location;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class FlywheelTeleop extends Command {
    private final FlywheelSubsystem kFlywheel;
    private final Supplier<Pose2d> kPose;
    private final Supplier<ChassisSpeeds> kVelocity;
    private final Supplier<Double> kAngularVelocity;
    private final Supplier<FieldMathHelpers.Location> kGetLocation;

    public FlywheelTeleop(
        FlywheelSubsystem mFlywheelSubsystem,
        Supplier<Pose2d> mPose,
        Supplier<ChassisSpeeds> mVelocity,
        Supplier<Double> mAngularVelocity,
        Supplier<FieldMathHelpers.Location> mGetLocation){
        kFlywheel = mFlywheelSubsystem;
        kPose = mPose;
        kVelocity = mVelocity;
        kAngularVelocity = mAngularVelocity;
        kGetLocation = mGetLocation;

        addRequirements(kFlywheel);
    }

    @Override
    public void execute(){
        double angle;
        double speed;
        double distance;
        switch (kGetLocation.get()) {
            case TRENCH: // Hide hood under trench
                kFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
                break;
            case ALLIANCE_ZONE: // Shoot to hub
                ChassisSpeeds speeds = kVelocity.get();
                distance = FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(
                    kPose.get(),
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    kAngularVelocity.get()).getNorm();
                distance = MathUtil.clamp(distance, FlywheelConstants.kDistanceMinimumInMeters, FlywheelConstants.kDistanceMaximumInMeters);
                speed = FlywheelConstants.kDistanceToFlywheelSpeed.get(distance);
                angle = FlywheelConstants.kDistanceToHoodAngle.get(distance);
                kFlywheel.setHoodTarget(angle);
                kFlywheel.setMode(FlywheelMode.SPINNING, speed);    
                break;
            default: // Default to passing mode
                kFlywheel.setHoodTarget(FlywheelConstants.kPassingAngle);
                distance = FieldMathHelpers.getTranslationToNearestPassingPoint(kPose.get()).getNorm();
                distance = MathUtil.clamp(distance, FlywheelConstants.kPassingMinimumInMeters, FlywheelConstants.kPassingMaximumInMeters);
                kFlywheel.setMode(FlywheelMode.SPINNING, FlywheelConstants.kPassingDistanceToSpeed.get(distance));
        }
    }
}