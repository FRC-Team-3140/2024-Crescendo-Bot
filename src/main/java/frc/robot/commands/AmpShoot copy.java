// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made AmpShoot the Y keybind on the xbox controller so we can test it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;

public class AmpShoot extends Command implements Constants{

    public IntakeShooter intakeShooter; 

    public AmpShoot() {
        intakeShooter = IntakeShooter.getInstance();

    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeShooter.setShooterVoltage(3);
  }

}