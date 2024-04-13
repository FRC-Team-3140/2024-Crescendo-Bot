// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1Commands;
// Working Good

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

/**
 * This command class represents the command to intake until a note is detected.
 * It sets the intake voltage to a predefined value and checks if a note is
 * detected.
 * The command ends when a note is detected or when interrupted.
 */
public class IntakeUntilNoteDetectedL1 extends Command {
  // Refrence to the intake shooter refrence
  Intake intake = Intake.getInstance();
  final double intakeVoltage = Constants.intakeVoltage;

  // tyler you need to go program the entire bot because itll probably be better
  // than pragya and channing.
  long startTime; // Called when the command is initially scheduled.

  double lastVoltage;
  /**
   * A command that controls the intake until a note is detected.
   * This command requires the intake subsystem.
   */
  public IntakeUntilNoteDetectedL1() {
    addRequirements(intake);
  }

  /**
   * Initializes the command.
   */
  @Override
  public void initialize() {
    intake.setIntakeVoltage(intakeVoltage);
  }

  /**
   * Executes the command.
   * Execute does nothing.
   */
  @Override
  public void execute() {
  }

  /**
   * This method is called when the command ends, either by completing or being interrupted.
   * It sets the intake voltage to 0 and schedules rumble for both controllers.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
    RobotContainer.controller.setRumble().schedule();
    RobotContainer.controller2.setRumble().schedule();
  }

  /**
   * Determines whether the command is finished or not.
   * The command is considered finished when the intake detects a note.
   *
   * @return true if the intake detects a note, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return intake.noteDetected();
    // return false;
  }

}
