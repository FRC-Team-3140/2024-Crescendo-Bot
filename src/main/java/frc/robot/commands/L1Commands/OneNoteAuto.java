// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command will set the arm to the angle and then shoot the speaker
 */
public class OneNoteAuto extends Command {
  private double shooterVoltage;
  private double intakeVoltage;


  /** Creates a new OneNoteAuto. */
  public OneNoteAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterVoltage = 9.6;
    this.intakeVoltage = 5;
  }

  /**
   * Initializes the command by setting the arm to the angle and then shooting the speaker
   */
  @Override
  public void initialize() {
    new SequentialCommandGroup(
      new SetArmToAngleL1(16),
      new ShootSpeakerL1(shooterVoltage, intakeVoltage)
    ).schedule();
  }
  
  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
