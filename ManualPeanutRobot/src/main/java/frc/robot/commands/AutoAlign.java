package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DynamicTuner;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    private boolean targetSeen;
    private DynamicTuner tuner;

    private final ProfiledPIDController controller;
    
    private final SwerveDriveSubsystem drive;
    private final VisionSubsystem vision;
    private final XboxController drivingController;
    private final MedianFilter mFilter;
    

    private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);
    private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);

    //private double mAngleToTarget;

    public AutoAlign(SwerveDriveSubsystem drive, VisionSubsystem vision, XboxController drivingController, int timeout) {
        this.tuner = DynamicTuner.getInstance();
        this.drive = drive;
        this.vision = vision;
        this.drivingController = drivingController;

        this.controller = new ProfiledPIDController(tuner.getAutoAlignP(), 0, tuner.getAutoAlignD(), 
        new TrapezoidProfile.Constraints(
            0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS, 
            0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS));
        

        mFilter = new MedianFilter(5);
        controller.enableContinuousInput(-180, 180);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetSeen = false;
        mFilter.reset();
        controller.reset(vision.getAngleToTarget());
    }

    @Override
    public void execute() {

        // Read joystick values
        double joyLeftY = -MathUtil.applyDeadband(drivingController.getLeftY(), 0.1);
        double joyLeftX = -MathUtil.applyDeadband(drivingController.getLeftX(), 0.1);
    
        // Apply controller profile curve for better low speed controllability 
        double fwdJoyVal = Math.signum(joyLeftY) * Constants.CONTROLLER_PROFLE_MAP.get(Math.abs(joyLeftY)); //Math.pow(Math.abs(joyLeftY), 2);
        double sideJoyVal = Math.signum(joyLeftX) * Constants.CONTROLLER_PROFLE_MAP.get(Math.abs(joyLeftX));//Math.pow(Math.abs(joyLeftX), 2);
    
        // Apply slew rate limiter to prevent sudden acceleration that can cause robot tipping
        fwdJoyVal = mFwdLimiter.calculate(fwdJoyVal);
        sideJoyVal = mSideLimiter.calculate(sideJoyVal);

        // Convert joystick inputs into translational velocities in m/s and turn velocities in rad/s
        double metersPerSecondFwd = Constants.MAXIMUM_SPEED_METERS_PER_SECONDS * fwdJoyVal;
        double metersPerSecondSide = Constants.MAXIMUM_SPEED_METERS_PER_SECONDS * sideJoyVal;
        double angleToTarget = vision.getAngleToTarget();
        double filteredAngleToTarget = mFilter.calculate(angleToTarget);

        SmartDashboard.putNumber("Angle To Target", angleToTarget);
        SmartDashboard.putNumber("Filtered Angle To Target", filteredAngleToTarget);

        if (targetSeen) {
            double rotationalVelocity = controller.calculate(filteredAngleToTarget, 0);
            double error = Math.abs(filteredAngleToTarget);
            if(error < 1.8) {
                rotationalVelocity = 0;
            }
            SmartDashboard.putNumber("Auto Rot Error", error);
            SmartDashboard.putNumber("Auto Rot", rotationalVelocity);

            // Include default drive logic to enable strafing while aiming
            drive.driveFieldOriented(metersPerSecondFwd, metersPerSecondSide, rotationalVelocity, false);
        } else {
            targetSeen = vision.targetsPresent();
        }
    }

    public void end() {
        System.out.println("Auto align ended");
    }
}