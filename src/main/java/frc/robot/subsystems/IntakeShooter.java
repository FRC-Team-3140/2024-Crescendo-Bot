// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Intake and Shooter related
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {

    public static IntakeShooter instance = null;

    /**
     * The motor that controls the intake.
     */
    public CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);

    /**
     * Flywheel 1 on the shooter.
     */
    public CANSparkMax shooterTop = new CANSparkMax(9, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 1's neo.
     */
    public RelativeEncoder encoderA = shooterTop.getEncoder();

    /**
     * Flywheel 2 on the shooter.
     */
    public CANSparkMax shooterBottom = new CANSparkMax(10, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 2's neo.
     */
    public RelativeEncoder encoderB = shooterBottom.getEncoder();

    private final DigitalInput peSensor = new DigitalInput(0);
    /**
     * True if a piece is in the intake.
     */
    public boolean holdingPiece = false;

    /**
     * Returns the instance.
     */
    public static synchronized IntakeShooter getInstance() {
        if (instance == null) {
            instance = new IntakeShooter();
        }
        return instance;
    }

    // Sets the motor to neutral on creation of the class.
    public IntakeShooter() {
        shooterTop.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        shooterTop.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        // determined
        shooterTop.restoreFactoryDefaults();
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTop.setInverted(false);
        shooterTop.setSmartCurrentLimit(30);
        shooterTop.burnFlash();

        shooterBottom.restoreFactoryDefaults();
        shooterBottom.follow(shooterTop);
        shooterBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        shooterBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        shooterBottom.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        shooterBottom.restoreFactoryDefaults();
        shooterBottom.setIdleMode(IdleMode.kCoast);
        shooterBottom.setInverted(true);
        shooterBottom.setSmartCurrentLimit(30);
        shooterBottom.burnFlash();

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.burnFlash();
    }

    /**
     * Sets the speed of the intake motor through voltage.
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void setHoldingPiece(boolean holdingPiece) {
        this.holdingPiece = holdingPiece;
    }

    /**
     * Sets the speed of the shooter's motor, make sure one is negative and one is
     * postive.
     */
    public void setShooterVoltage(double voltage) {
        shooterTop.setVoltage(voltage);
        shooterBottom.setVoltage(-voltage);
    }
    public void setShooterVoltageTop(double voltage) {
        shooterTop.setVoltage(voltage+.15);
    }
    public void setShooterVoltageBottom(double voltage) {
        shooterBottom.setVoltage(-voltage);
    }

    PIDController topController = new PIDController(.0018, 0, .00006);
    PIDController bottomController = new PIDController(.0018, 0, .00006);
    //TODO: Calibrate values
    public void setShooterSpeed(double speed) {
        topController.setSetpoint(speed);
        bottomController.setSetpoint(-speed);
        shooterTop.setVoltage(topController.calculate(shooterTop.getEncoder().getVelocity()));
        shooterBottom.setVoltage(bottomController.calculate(shooterBottom.getEncoder().getVelocity()));

    }

       public double getShooterSpeed() {
        return Math.min(-shooterBottom.getEncoder().getVelocity(), shooterTop.getEncoder().getVelocity());
    }

    public boolean hasNote() {
        return holdingPiece;
    }

    public boolean noteDetected() {
        return proximityThresholdExeeded;
    }

    /**
     * True when a note reaches the sensor.
     */
    public boolean proximityThresholdExeeded;

    @Override
    public void periodic() {
        // The method getProximity() returns a value 0 - 2047, with the closest being .
        proximityThresholdExeeded = !peSensor.get();
        if (!peSensor.get()) {
            // System.out.println("p44e" + !peSensor.get());
        }
        // Open Smart Dashboard to see the color detected by the sensor.
        SmartDashboard.putBoolean("Has Note", holdingPiece);
        SmartDashboard.putBoolean("NoteDetected", proximityThresholdExeeded);
        SmartDashboard.putNumber("SpeedTop", shooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("SpeedBottom", -shooterBottom.getEncoder().getVelocity());
    }   
}