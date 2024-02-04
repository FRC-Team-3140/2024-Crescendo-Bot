// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made SpeakerShoot the X keybind on the xbox controller so we can test it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class SpeakerShoot extends Command implements Constants {

    public IntakeShooter intakeShooter; 
    private InterpolatingDoubleTreeMap voltageInterpolator;

    public SpeakerShoot() {
        intakeShooter = IntakeShooter.getInstance();
        voltageInterpolator = new InterpolatingDoubleTreeMap(/* Add your inverseInterpolator, interpolator, and comparator here */);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Use the InterpolatingTreeMap to get the interpolated voltage for the key (e.g., joystick position)
        // PLEASE IGNORE the joystick part, this will be connected to the camera but its not ready yet
        double joystickPosition = 0; /* Get your joystick position here */;
        double interpolatedVoltage = voltageInterpolator.get(joystickPosition);
        
        // Set the shooter voltage based on the interpolated value
        intakeShooter.setShooterVoltage(interpolatedVoltage);
    }

    // Other methods for isFinished(), end(), etc., can be added if needed.
}
