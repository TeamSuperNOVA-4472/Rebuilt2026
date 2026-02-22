package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldMathHelpers;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelMode;

public class calculateFlywheelSpeed extends Command 
{
    private final FlywheelSubsystem kFlywheel;

    private final FlywheelMode kFlyMode;

    private final Supplier<Pose2d> position;

    private final Supplier<ChassisSpeeds> velocities;

    private final InterpolatingDoubleTreeMap kDistanceToFinalSpeed = new InterpolatingDoubleTreeMap();

    private double theSpeed = 0;

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
        /*
        Pose2d pose = position.get();

        ChassisSpeeds speeds = velocities.get();

        Translation2d toHub = FieldMathHelpers.getTranslationToHub(pose);

        double theDistance = toHub.getNorm();

        double theTime = theDistance / theSpeed;

        Translation2d distanceTheRobotMovesWhileBallIsInTheAir = new Translation2d(speeds.vxMetersPerSecond * theTime, speeds.vyMetersPerSecond * theTime);

        Translation2d leading = toHub.minus(distanceTheRobotMovesWhileBallIsInTheAir);

        double theChangingDistance = leading.getNorm();

        double theTargetSpeed = kDistanceToFinalSpeed.get(theChangingDistance);

        kFlywheel.setMode(kFlyMode);

        kFlywheel.setTargetSpeed(theTargetSpeed);
        */
    }

    @Override
    public void execute()
    {
        Pose2d pose = position.get();

        ChassisSpeeds speeds = velocities.get();

        Translation2d toHub = FieldMathHelpers.getTranslationToHub(pose);

        double theDistance = toHub.getNorm();

        double theTime = theDistance / theSpeed;

        Translation2d distanceTheRobotMovesWhileBallIsInTheAir = new Translation2d(speeds.vxMetersPerSecond * theTime, speeds.vyMetersPerSecond * theTime);

        Translation2d leading = toHub.minus(distanceTheRobotMovesWhileBallIsInTheAir);

        double theChangingDistance = leading.getNorm();

        double theTargetSpeed = kDistanceToFinalSpeed.get(theChangingDistance);

        kFlywheel.setMode(kFlyMode);

        kFlywheel.setTargetSpeed(theTargetSpeed);
    }
}
