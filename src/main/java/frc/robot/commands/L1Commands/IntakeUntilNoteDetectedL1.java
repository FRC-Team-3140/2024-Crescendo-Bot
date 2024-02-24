// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.L1Commands;
// Working Good

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;

public class IntakeUntilNoteDetectedL1 extends Command {
  // Refrence to the intake shooter refrence
  IntakeShooter intakeShooter = IntakeShooter.getInstance();
  final double intakeVoltage = Constants.intakeVoltage;
  static PowerDistribution pdp = new PowerDistribution(55, ModuleType.kRev);

  /** Creates a new IntakeUntilNoteDetected. */
  public IntakeUntilNoteDetectedL1() {
    addRequirements(intakeShooter);
  }

  long startTime; // Called when the command is initially scheduled.
  double lastVoltage;

  @Override
  public void initialize() {
    // intakeShooter.setHoldingPiece(false);
    // lastVoltage = pdp.getCurrent(17);
    // startTime = System.currentTimeMillis();
    intakeShooter.setIntakeVoltage(intakeVoltage);
  }

  public void execute() {

    // Starts running the intake at a slower speed when there is a current spike;
    // The color sensor takes some time to recognize it.
    // if(pdp.getCurrent(17) > 5 && System.currentTimeMillis() - startTime > 1000){
    // intakeShooter.setIntakeVoltage(0);

    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.setHoldingPiece(true);
    
    intakeShooter.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeShooter.noteDetected();
    // return false;
  }

}
