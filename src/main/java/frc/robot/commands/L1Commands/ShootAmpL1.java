// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made AmpShoot the Y keybind on the xbox controller so we can test it

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
//Check to make sure this works
import frc.robot.subsystems.Shooter;

/**
 * The AmpShoot class represents a command that runs the intake and shooter at
 * the right speed to shoot the note into the amp.
 * It sets the intake and shooter voltages to a specific value when initialized,
 * and sets them back to zero when the command ends.
 */
public class ShootAmpL1 extends Command implements Constants {

  public Intake intake;
  public Shooter shooter;

  /**
   * A command that shoots the note with increased power and speed.
   */
  public ShootAmpL1() {
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(intake, shooter);
  }

// TODO move to head of class
  long startTime;

  /**
   * Initializes the ShootAmpL1 command.
   * Sets the start time, intake voltage, and shooter voltage.
   */
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    intake.setIntakeVoltage(3);
    shooter.setShooterVoltage(3);
  }

  /**
   * Determines whether the command has finished executing.
   * 
   * @return true if the command has finished, false otherwise
   */
  @Override
  public boolean isFinished() {
    // TODO This should automatically terminate after a certain amount of time - DB
    // TODO: I dont think the channel or the current it is greater than is correct. Please check that
    return System.currentTimeMillis() - startTime > 2000; //|| IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 7; 
  }

  /**
   * This method is called when the command ends.
   * It sets the intake voltage and shooter voltage to 0.
   * 
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
    shooter.setShooterVoltage(0);
  }

}
