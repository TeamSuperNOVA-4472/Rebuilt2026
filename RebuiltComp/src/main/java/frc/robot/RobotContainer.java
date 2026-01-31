// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.setIntake;
import frc.robot.Commands.setSpindexer;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.IntakeMode;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class RobotContainer {
  IntakeSubsystem mIntake;
  SpindexerSubsystem mSpindexer;
  XboxController mDriver;
  public RobotContainer() {
    mIntake = IntakeSubsystem.kIntake;
    mSpindexer = SpindexerSubsystem.kInstance;
    mDriver = new XboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    Trigger intakeToggle = new Trigger(mDriver::getLeftBumperButtonPressed);
    intakeToggle.onTrue(new setIntake(mIntake, IntakeMode.INTAKE));
    Trigger outtakeToggle = new Trigger(mDriver::getRightBumperButton);
    outtakeToggle.onTrue(new setIntake(mIntake, IntakeMode.OUTTAKE));
    Trigger spindexerHold = new Trigger(mDriver::getAButton);
    spindexerHold.onTrue(new setSpindexer(mSpindexer, SpindexerMode.LOAD));
    spindexerHold.onFalse(new setSpindexer(mSpindexer, SpindexerMode.OFF));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
