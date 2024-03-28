// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made AmpShoot the Y keybind on the xbox controller so we can test it

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
//Check to make sure this works
import frc.robot.subsystems.Shooter;

/**
 * The AmpShoot class represents a command that runs the intake and shooter at
 * the right speed to shoot the note into the amp.
 * It sets the intake and shooter voltages to a specific value when initialized,
 * and sets them back to zero when the command ends.
 */
public class ShootAmpL1 extends Command {

  public Intake intake;
  public Shooter shooter;

  public ShootAmpL1() {
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(intake, shooter);
  }

  /**
   * Initializes the AmpShootL1 command.
   * Sets the intake and shooter voltages.
   */
  long startTime;

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
    return System.currentTimeMillis() - startTime > 5000; // || IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 7;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
    shooter.setShooterVoltage(0);
  }

}
