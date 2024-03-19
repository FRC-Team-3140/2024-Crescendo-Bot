// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made DefaultShoot the B keybind on the Xbox controller so we can test it
// This command is meant so that we can input the speeds

package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** Set the shooter speed to ~max and then shoot the note at the speaker. */
public class ShootSpeakerL1 extends Command implements Constants {

    private final Shooter shooter;
    private final Intake intake;
    private final double shooterSpeed;
    private final double voltage2;
    private double freeSpeed;
    private double deadband = 20;

    public ShootSpeakerL1(double shooterSpeed, double intakeVoltage) {
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.shooterSpeed = shooterSpeed;
        this.voltage2 = intakeVoltage;
        addRequirements(intake, shooter);
        freeSpeed = shooterSpeed - deadband;
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
        shooter.setShooterVoltage(shooterSpeed);
    }

    double timeSinceSpinUp = Double.MAX_VALUE;
    boolean hitSpeed = false;

    @Override
    public void execute() {
        // shooter.setShooterSpeed(shooterSpeed);
        if(shooter.getShooterSpeed() >= freeSpeed && !hitSpeed){
            hitSpeed = true;
            timeSinceSpinUp = System.currentTimeMillis();
            RobotContainer.controller2.setRumble().schedule();
        }
        if(System.currentTimeMillis() - timeSinceSpinUp > 300 && shooter.getShooterSpeed() >= freeSpeed){
            intake.setIntakeVoltage(voltage2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // intakeShooter.setIntakeVoltage(0);
        // intakeShooter.setShooterVoltage(0);

    }

    @Override
    public boolean isFinished() {
        
        return System.currentTimeMillis() - timeSinceSpinUp > 600 && shooter.getShooterSpeed() >= freeSpeed;
        // return System.currentTimeMillis() - startTime > 3000 ;//||
        // IntakeUntilNoteDetectedL1.pdp.getCurrent(17) > 5;//I dont think the channel
        // or the current it is greater than is correct. Please check that
    }

}
