// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Intake and Shooter related
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Color sensor related
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class IntakeShooter extends SubsystemBase {

    public static IntakeShooter instance = null;

    /**
     * The motor that controls the intake.
     */
    public CANSparkMax intakeMotor = new CANSparkMax(20, MotorType.kBrushless);

    /**
     * Flywheel 1 on the shooter.
     */
    public CANSparkMax shooterA = new CANSparkMax(9, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 1's neo.
     */
    public RelativeEncoder encoderA = shooterA.getEncoder();

    /**
     * Flywheel 2 on the shooter.
     */
    public CANSparkMax shooterB = new CANSparkMax(8, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 2's neo.
     */
    public RelativeEncoder encoderB = shooterB.getEncoder();

    // Change the I2C port below to match the connection of the color sensor.
    /**
     * The I2C port the proximity sensor is connected to.
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /** 
     * This object is constructed with an I2C port as a parameter.
     * This device will be automatically initialized with default parameters.
     */
    private final ColorSensorV3 proximitySensor = new ColorSensorV3(i2cPort);

    /**
     * True if a piece is in the intake.
     */
    public static boolean holdingPiece = false;

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
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /** 
     * Sets the speed of the intake motor through voltage.
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /** 
     * Sets the speed of the shooter's motor, make sure one is negative and one is postive.
     */
    public void setShooterVoltage(double voltage) {
        shooterA.setVoltage(-voltage);
        shooterB.setVoltage(voltage);
    }
    
    /** 
     * True when a note reaches the sensor.
     */
    public static boolean proximityThresholdExeeded;

    @Override
    public void periodic() {
        
        // The method getProximity() returns a value 0 - 2047, with the closest being .
        int detectedProximity = proximitySensor.getProximity();

        //Open Smart Dashboard to see the color detected by the sensor.
        SmartDashboard.putNumber("Proximity", detectedProximity);
        SmartDashboard.putBoolean("NoteDetected", proximityThresholdExeeded);
    }
//fuck you grace - joseph
}
