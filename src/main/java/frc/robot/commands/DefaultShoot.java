// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made DefaultShoot the B keybind on the Xbox controller so we can test it
// This command is meant so that we can input the speeds

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;

public class DefaultShoot extends Command implements Constants {

    private final IntakeShooter intakeShooter;
    private final double voltage;
    private final double voltage2;

    public DefaultShoot(double shooterVoltage, double intakeVoltage) {
        this.intakeShooter = IntakeShooter.getInstance();
        this.voltage = shooterVoltage;
        this.voltage2 = intakeVoltage;
        addRequirements(intakeShooter);
      // Adjust the desiredVoltage variable to the voltage value you want to use. 
      // You can then use this instance of DefaultShoot in your robot's command scheduler or bind it to a button as needed for your specific control setup.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        new SequentialCommandGroup(
        new InstantCommand(() -> {intakeShooter.setShooterVoltage(voltage);}),
        new WaitCommand(2), //Spin up longer after 114 inches 
        new InstantCommand(()->{intakeShooter.setIntakeVoltage(voltage2);})
        ).schedule();
    }
    
}
