package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class TestProfiledPidCommand extends Command {

    private final SwerveDriveSubsystem mSwerveDriveSubsystem;

    private final ProfiledPIDController mProfiledPIDController =
        new ProfiledPIDController(.15, 0, 0.005, 
            new TrapezoidProfile.Constraints(
                0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS, 
                0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS));
    private double mTargetAngle = 0;

    public TestProfiledPidCommand(SwerveDriveSubsystem pSwerveDriveSubsystem) {
        mSwerveDriveSubsystem = pSwerveDriveSubsystem;
        mProfiledPIDController.enableContinuousInput(0, 360);
        addRequirements(pSwerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        double currentAngle = mSwerveDriveSubsystem.getPose().getRotation().getDegrees();
        mTargetAngle = (((currentAngle + 90) % 360) + 360) % 360;
        mProfiledPIDController.reset(currentAngle);
    }

    @Override
    public void execute() {
        double currentAngle = ((mSwerveDriveSubsystem.getPose().getRotation().getDegrees() % 360) + 360) % 360;

        double output = mProfiledPIDController.calculate(currentAngle, mTargetAngle);
        double diff = Math.abs(mTargetAngle - currentAngle);
        double error = diff;
        if(diff > 180) {
            error = 360 - diff; 
        }
        SmartDashboard.putNumber("Profiled PID target", mTargetAngle);
        SmartDashboard.putNumber("Profiled PID current", currentAngle);
        SmartDashboard.putNumber("Profiled PID Error", error);
        SmartDashboard.putNumber("Profiled PID Output", output);
        if(error < .5) {
            output = 0; 
        }
        mSwerveDriveSubsystem
        .driveRobotOriented(
            0, 
            0, 
            output, 
            false);
    }
}
