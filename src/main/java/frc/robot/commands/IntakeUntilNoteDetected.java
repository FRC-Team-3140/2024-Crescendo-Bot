// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;

public class IntakeUntilNoteDetected extends Command {
  // Refrence to the intake shooter refrence
  IntakeShooter intakeShooter = IntakeShooter.getInstance();
  final double intakeVoltage = Constants.intakeVoltage; 
  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new IntakeUntilNoteDetected. */
  public IntakeUntilNoteDetected() {
    addRequirements(intakeShooter);
   }
  long startTime;  // Called when the command is initially scheduled.
  double lastVoltage;
  @Override
  public void initialize() {
    lastVoltage = pdp.getCurrent(17);
    startTime = System.currentTimeMillis();
    intakeShooter.setIntakeVoltage(intakeVoltage);
  }
  public void execute(){

    //Starts running the intake at a slower speed when there is a current spike; The color sensor takes some time to recognize it. 
    if(pdp.getCurrent(17) > 4.5 && System.currentTimeMillis() - startTime > 1000){
      intakeShooter.setIntakeVoltage(0);
      
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // holding piece is static, so it is refrenced staticly.
    // !interrupted makes it false when it is manually shut off, but true when it ends due to the sensor
    IntakeShooter.holdingPiece = !interrupted;

    // this method isn't, so it is called via the local refrence
    intakeShooter.setIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IntakeShooter.proximityThresholdExeeded;
    // return false;
  }

}
