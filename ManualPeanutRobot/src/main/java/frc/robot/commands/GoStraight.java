package frc.robot.commands;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class GoStraight extends Command {
    
    private final SwerveDriveSubsystem drive;
    private final ProfiledPIDController controller;
    private final PIDController mGyroController;

    private double mDistance;
    private final double[] initialPositions;
    private final double mSign;

    private boolean mFinished = false;

    public GoStraight(SwerveDriveSubsystem driveSubsystem, double speed, double distance) {
        drive = driveSubsystem;
        controller = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(speed, 1.0));
        mGyroController = new PIDController(0.15, 0, 0.00);
        mGyroController.enableContinuousInput(0, 360);
        mDistance = distance;
        initialPositions = new double[driveSubsystem.getModulePositions().length];
        mSign = distance < 0 ? -1 : 1;
        addRequirements(drive);
    }

    public void setDistance(double pDistance) {
        mDistance = pDistance;
    }

    public double getError() {
        return mDistance - getDistanceTraveled();
    }

    public double getDistanceTraveled() {
        double total = 0;
        double avgDistance;
        SwerveModulePosition[] currentPositions = drive.getModulePositions();

        for (int i = 0; i < currentPositions.length; i ++) {
            double moduleDistTraveled = Math.abs(currentPositions[i].distanceMeters - initialPositions[i]);
            total += moduleDistTraveled;
        }
        avgDistance = total / (double) drive.getModulePositions().length;
        return mSign * avgDistance;
    }

    @Override
    public void initialize() {
        for (int i = 0; i < drive.getModulePositions().length; i++) {
            initialPositions[i] = drive.getModulePositions()[i].distanceMeters;
        }
        controller.reset(0);
        controller.setGoal(mDistance);
        mGyroController.setSetpoint(drive.getRobotGyroHeading());
        mFinished = false;
    }

    @Override
    public void execute() {
        double errorDist = getError() / mDistance;
        double speed = 0;
        double turn = 0;
        if(errorDist > 0.01 ) {
          speed = controller.calculate(getDistanceTraveled());
          turn = mGyroController.calculate(drive.getRobotGyroHeading());
        } else {
            mFinished = true;
        }
        drive.driveRobotOriented(speed, 0, turn, false);
        SmartDashboard.putNumber("Drive forward error", errorDist);
        SmartDashboard.putNumber("current speed", speed);
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }
}