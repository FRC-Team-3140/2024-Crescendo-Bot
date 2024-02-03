// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAndShooter;

public class SpeakerShoot extends CommandBase {

    private final SpeakerShoot IntakeAndShooter;
    private final XboxController xboxController;

    public SpeakerShoot(SpeakerShoot IntakeAndShooter, XboxController xboxController) {
        this.SpeakerShoot = IntakeAndShooter;
        this.xboxController = xboxController;
        addRequirements(IntakeAndShooter);
    }

    @Override
    public void initialize() {
        // You can add any initialization logic here.
    }

// Using Y to test this code

    @Override
    public void execute() {
        // Check if the Y button on the Xbox controller is pressed
        if (xboxController.getYButtonPressed()) {
            // Shoot at high speed into the speaker
            SpeakerShoot.shoot(1.0); // Adjust power as needed
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooting when the command ends
        SpeakerShoot.shoot(0.0);
    }

    @Override
    public boolean isFinished() {
        // This command finishes immediately, as it is meant for instantaneous shooting.
        return true;
    }
}
