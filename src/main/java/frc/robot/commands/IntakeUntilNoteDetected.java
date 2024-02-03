// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;

public class IntakeUntilNoteDetected extends Command {
  // Refrence to the intake shooter refrence
  IntakeShooter intakeShooter = IntakeShooter.getInstance();
  final double intakeVoltage = Constants.intakeVoltage; 
  /** Creates a new IntakeUntilNoteDetected. */
  public IntakeUntilNoteDetected() { }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeShooter.setIntakeVoltage(intakeVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IntakeShooter.proximityThresholdExeeded;
  }

}
