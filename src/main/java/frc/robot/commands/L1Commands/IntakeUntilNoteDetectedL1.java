// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1Commands;
// Working Good

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


/**
 * This command will intake until a note is detected by the intake
 */
public class IntakeUntilNoteDetectedL1 extends Command {
  // Refrence to the intake shooter refrence
  Intake intake = Intake.getInstance();
  Shooter shooter = Shooter.getInstance();
  final double intakeVoltage = Constants.intakeVoltage;

  /** Creates a new IntakeUntilNoteDetected. */
  public IntakeUntilNoteDetectedL1() {
    addRequirements(intake, shooter);
  }

  // TODO: Move to head of class.
  long startTime; // Called when the command is initially scheduled.
  double lastVoltage;

  /**
   * Initializes the command by setting the intake voltage and shooter voltage.
   */
  @Override
  public void initialize() {
    intake.setIntakeVoltage(intakeVoltage);
    shooter.setShooterVoltage(0);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {}

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return intake.noteDetected();
    // return false;
  }

}
