// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

/**
 * This class represents a command for autonomous mode that performs a specific sequence of actions.
 * It sets the arm to a specific angle and then shoots a ball using the shooter and intake subsystems.
 */
public class OneNoteAuto extends Command {
  private double shooterVoltage;
  private double intakeVoltage;

  /**
   * This class represents a command for performing a specific autonomous routine called "OneNoteAuto".
   * It sets the shooter voltage and intake voltage to their respective constants defined in the Constants class.
   */
  public OneNoteAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterVoltage = Constants.shooterVoltage;
    this.intakeVoltage = Constants.intakeVoltage;
  }

  /**
   * Initializes the OneNoteAuto command.
   * This method is called once when the command is first scheduled.
   */
  @Override
  public void initialize() {
    new SequentialCommandGroup(
        new SetArmToAngleL1(16),
        new ShootSpeakerL1(shooterVoltage, intakeVoltage)).schedule();
  }

  /**
   * Executes the command.
   */
  @Override
  public void execute() {
  }

  /**
   * This method is called when the command ends.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Determines whether the command has finished executing.
   * 
   * @return true if the command has finished, false otherwise
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
