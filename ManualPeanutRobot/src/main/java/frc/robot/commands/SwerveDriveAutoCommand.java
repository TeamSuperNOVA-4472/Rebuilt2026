package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveDriveAutoCommand extends SequentialCommandGroup {
    private static final String PATH_FILE = "Sideways.wpilib.json";

    public SwerveDriveAutoCommand(SwerveDriveSubsystem pSwerve) {
        SwerveDriveKinematics kinematics = pSwerve.getSwerveKinematics();
        TrajectoryConfig config = new TrajectoryConfig(1.0, 0.5)
                                        .setKinematics(kinematics);
        

        Trajectory parsedTrajectory =
             TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),
                                                    List.of(new Translation2d(0.25,0.25)),  
                                                    new Pose2d(0.5,0.5, new Rotation2d()), 
                                                    config);
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(PATH_FILE);
            parsedTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + PATH_FILE, ex.getStackTrace());
        }

        Trajectory swerveTrajectory = parsedTrajectory;
        ProfiledPIDController thetaController = new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(swerveTrajectory, 
                                                                                      pSwerve::getPose, 
                                                                                      kinematics, 
                                                                                      new PIDController(1.0, 0.0, 0.0), 
                                                                                      new PIDController(1.0, 0.0, 0.0), 
                                                                                      thetaController, 
                                                                                      pSwerve::setModuleStates, 
                                                                                      pSwerve);
        addCommands(new InstantCommand(() -> {
                        //pSwerve.reset();
                        pSwerve.resetOdometry(swerveTrajectory.getInitialPose());
                        pSwerve.setSkipInitialization(true);
                    }),
                    swerveControllerCommand,
                    new InstantCommand(() ->
                        pSwerve.driveFieldOriented(
                            0,
                            0, 
                            0, 
                            false)),
                    new WaitCommand(15));
    }
}
