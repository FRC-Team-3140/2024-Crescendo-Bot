// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made DefaultShoot the B keybind on the Xbox controller so we can test it
// This command is meant so that we can input the speeds

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


/**
 * Represents a command to shoot the speaker in the L1 position.
 * This command sets the shooter speed and intake voltage, and adds the intake and shooter subsystems as requirements.
 * The desired voltage can be adjusted using the `shooterVoltage` parameter.
 * This command can be used in the robot's command scheduler or bound to a button for control.
 * 
 * @param shooterVoltage the voltage to set the shooter to (in volts)
 * @param intakeVoltage the voltage to set the intake to (in volts)
 */
public class ShootSpeakerL1 extends Command {

    private final Shooter shooter;
    private final Intake intake;
    private final double shooterSpeed;
    private final double voltage2;
    private double freeSpeed;
    private double deadband = 1000;

    // Called when the command is initially scheduled.
    SequentialCommandGroup test;

    long startTime;
    double timeSinceSpinUp = Double.MAX_VALUE;

    boolean hitSpeed = false;

    double timeSinceIntakeSpinUp = Double.MAX_VALUE;

    /**
     * Represents a command to shoot the speaker in the L1 position.
     * This command sets the shooter speed and intake voltage, and adds the intake and shooter subsystems as requirements.
     * The desired voltage can be adjusted using the `shooterVoltage` parameter.
     * This command can be used in the robot's command scheduler or bound to a button for control.
     * 
     * @param shooterVoltage the voltage to set the shooter to (in volts)
     * @param intakeVoltage the voltage to set the intake to (in volts)
     */
    public ShootSpeakerL1(double shooterVoltage, double intakeVoltage) {
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.shooterSpeed = shooterVoltage;
        this.voltage2 = intakeVoltage;
        addRequirements(intake, shooter);
        freeSpeed = (473 * shooterSpeed) - deadband;
        // Adjust the desiredVoltage variable to the voltage value you want to use.
        // You can then use this instance of DefaultShoot in your robot's command
        // scheduler or bind it to a button as needed for your specific control setup.
    }

    /**
     * Initializes the ShootSpeakerL1 command.
     * Sets the shooter voltage for the top and bottom motors to the specified shooter speed.
     */
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        // TODO: Recommend using encoders and PID to control the shooter speed. Much
        // more consistant shots. See notes in IntakeShooter. -DB
        shooter.setShooterVoltageTop(shooterSpeed);
        shooter.setShooterVoltageBottom(shooterSpeed);
    }

    /**
     * Executes the shoot speaker command.
     * If the shooter speed is greater than or equal to the free speed and the hit speed flag is false,
     * it sets the hit speed flag to true and records the current time.
     * If the time since spin up is greater than 300 milliseconds and the shooter speed is still greater than or equal to the free speed,
     * it records the current time and sets the intake voltage to voltage2.
     */
    @Override
    public void execute() {
        if (shooter.getShooterSpeed() >= freeSpeed && !hitSpeed) {
            hitSpeed = true;
            System.out.println("Hit Speed");
            timeSinceSpinUp = System.currentTimeMillis();
            // RobotContainer.controller2.setRumble().schedule();
        }
        if ((System.currentTimeMillis() - timeSinceSpinUp) > 300 && shooter.getShooterSpeed() >= freeSpeed) {
            timeSinceIntakeSpinUp = System.currentTimeMillis();
            intake.setIntakeVoltage(voltage2);
        }
    }

    /**
     * This method is called when the command ends.
     * It stops the intake and shooter by setting their voltages to 0.
     * 
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVoltage(0);
        shooter.setShooterVoltage(0);
    }

    /**
     * Determines whether the command is finished or not.
     * The command is considered finished if the time since the intake spin-up is greater than 600 milliseconds
     * and the shooter speed is greater than or equal to the free speed.
     *
     * @return true if the command is finished, false otherwise
     */
    @Override
    public boolean isFinished() {

        return System.currentTimeMillis() - timeSinceIntakeSpinUp > 600 && shooter.getShooterSpeed() >= freeSpeed;
        // return System.currentTimeMillis() - startTime > 3000 ;//||
        // IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 5;//I dont think the channel
        // or the current it is greater than is correct. Please check that
    }

}
