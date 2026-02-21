// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.FlywheelTestCommand;
import frc.robot.commands.GoToAngleCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.ToggleSpindexer;
import frc.robot.commands.calculateFlywheelSpeed;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelMode;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  private final XboxController mDriver =
      new XboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem mVisionSubsystem;

  private final FlywheelSubsystem kFlywheel = FlywheelSubsystem.kFlywheel;
  private final SpindexerSubsystem kSpindexer = SpindexerSubsystem.kSpindexer;

  private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1.0);

  private final Trigger mResetPose = new Trigger(mDriver::getYButton);

  private int flywheelSpeed = 0;


  private final SwerveTeleop mSwerveTeleop = new SwerveTeleop(
    () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())), 
    () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
    () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
    mDriver::getAButton,
    mDriver::getXButton,
    mSwerveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
    NamedCommands.registerCommand("DoTheThingCommand", new DoTheThingCommand());
    new EventTrigger("TheEvent").onTrue(
      new InstantCommand(() -> System.out.println("The Event has triggered")));

      mVisionSubsystem = new VisionSubsystem(mSwerveSubsystem::getHeadingDegrees, mSwerveSubsystem::getAngularVelocity,
      (PoseEstimate pose, Matrix<N3, N1> stdDevs) -> {
        mSwerveSubsystem.addVisionMeasurement(pose.pose, pose.timestampSeconds, stdDevs);
      }
      );

    Trigger flyWheelToggle = new Trigger(mDriver::getLeftBumperButtonPressed);
    flyWheelToggle.onTrue(new InstantCommand(() -> {
      kFlywheel.setMode(FlywheelMode.SPINNING);
      Pose2d botpose = mSwerveSubsystem.getPose();
      ChassisSpeeds botSpeeds = mSwerveSubsystem.getFieldRelativeSpeeds();
      double flywheelSpeed = Constants.FlywheelConstants.kDistanceToVelocity.get((FieldMathHelpers.getDistanceToHub(mSwerveSubsystem.getPose())));
      double speed = FieldMathHelpers.getFlywheelSpeedWithSomeSpeedInDegrees(botpose, botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond, flywheelSpeed);
      kFlywheel.setTargetSpeed(Constants.FlywheelConstants.kDistanceToVelocity.get((FieldMathHelpers.getDistanceToHub(mSwerveSubsystem.getPose()))));
    }));

    Trigger flyWheelToggleOff = new Trigger(mDriver::getRightBumperButtonPressed);
    flyWheelToggleOff.onTrue(new InstantCommand(() -> {
      kFlywheel.setMode(FlywheelMode.OFF);
      kFlywheel.setTargetSpeed(0);
    }));

    Trigger spindexerToggle = new Trigger(mDriver::getLeftStickButton);
    spindexerToggle.onTrue(new ToggleSpindexer(kSpindexer));
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try {
      return new FlywheelTestCommand(mSwerveSubsystem, kFlywheel, kSpindexer);
    } catch (AutoBuilderException e) {
      return new InstantCommand();
    }
  }

}
