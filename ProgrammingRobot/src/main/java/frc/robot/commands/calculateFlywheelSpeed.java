package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldMathHelpers;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelMode;

public class calculateFlywheelSpeed extends Command 
{
    private final FlywheelSubsystem kFlywheel;

    private final FlywheelMode kFlyMode;

    private final Supplier<Pose2d> position;

    private final Supplier<ChassisSpeeds> velocities;

    public calculateFlywheelSpeed
    (
        FlywheelSubsystem mFlywheel,
        Supplier<Pose2d> mPosition,
        Supplier<ChassisSpeeds> mVelocities,
        FlywheelMode mFlyMode
    )
    {
        kFlywheel = mFlywheel;

        kFlyMode = mFlyMode;

        position = mPosition;

        velocities = mVelocities;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize()
    {
        kFlywheel.setMode(FlywheelMode.SPINNING);
    }

    @Override
    public void execute()
    {
        Pose2d pose = position.get();

        ChassisSpeeds speeds = velocities.get();

        Translation2d translationToHub = FieldMathHelpers.getTranslation2dToHubWithSomeSpeed(pose, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        double theChangingDistance = translationToHub.getNorm();

        double theTargetSpeed = Constants.FlywheelConstants.kDistanceToVelocity.get(theChangingDistance);
        SmartDashboard.putNumber("Get flywheel speed: ", theTargetSpeed);

        kFlywheel.setTargetSpeed(theTargetSpeed);
    }
}
