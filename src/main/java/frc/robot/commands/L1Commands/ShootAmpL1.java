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

  long startTime;


  /**
   * A command that shoots the ball with increased power and speed.
   */
  public ShootAmpL1() {
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(intake, shooter);
  }

  /**
   * Initializes the ShootAmpL1 command.
   * Sets the intake and shooter voltages to 3.
   */
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    intake.setIntakeVoltage(3);
    shooter.setShooterVoltage(6);
  }

  /**
   * Executes the ShootAmpL1 command.
   * This method does nothing, as the intake and shooter voltages are set in the initialize method.
   */
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > 5000; // || IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 7;
  }

  /**
   * This method is called when the command ends.
   * It sets the intake and shooter voltages to 0.
   * 
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
    shooter.setShooterVoltage(0);
  }

}
