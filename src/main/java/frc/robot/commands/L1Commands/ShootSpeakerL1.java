// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made DefaultShoot the B keybind on the Xbox controller so we can test it
// This command is meant so that we can input the speeds

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;
//Works Well

/** Set the shooter speed to ~max and then shoot the note at the speaker. */
public class ShootSpeakerL1 extends Command implements Constants {

    private final IntakeShooter intakeShooter;
    private final double voltage;
    private final double voltage2;

    public ShootSpeakerL1(double shooterVoltage, double intakeVoltage) {
        this.intakeShooter = IntakeShooter.getInstance();
        this.voltage = shooterVoltage;
        this.voltage2 = intakeVoltage;
        addRequirements(intakeShooter);
        // Adjust the desiredVoltage variable to the voltage value you want to use.
        // You can then use this instance of DefaultShoot in your robot's command
        // scheduler or bind it to a button as needed for your specific control setup.
    }

    // Called when the command is initially scheduled.
    SequentialCommandGroup test;
    long startTime;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        // TODO: Recommend using encoders and PID to control the shooter speed. Much
        // more consistant shots. See notes in IntakeShooter. -DB
        intakeShooter.setShooterVoltage(voltage);
    }

    @Override
    public void execute() {
        if (intakeShooter.getShooterSpeed() >= 4600) {
            intakeShooter.setIntakeVoltage(voltage2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeShooter.setIntakeVoltage(0);
        intakeShooter.setShooterVoltage(0);

    }

    @Override
    public boolean isFinished() {
        return false;
        // return System.currentTimeMillis() - startTime > 3000 ;//||
        // IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 5;//I dont think the channel
        // or the current it is greater than is correct. Please check that
    }

}
