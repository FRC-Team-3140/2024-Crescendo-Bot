package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake subsystem controls the intake mechanism of the robot.
 * It includes methods for setting the intake voltage and detecting if a note is
 * detected.
 */
public class Intake extends SubsystemBase {
    public CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final DigitalInput peSensor = new DigitalInput(0);
    public static Intake intake = new Intake();

    /**
     * The Intake class represents the intake subsystem of the robot.
     * It is responsible for controlling the intake motor and its settings.
     */
    private Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.burnFlash();
    }

    /**
     * Returns the singleton instance of the Intake class.
     * If the instance does not exist, it creates a new one.
     *
     * @return the singleton instance of the Intake class
     */
    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    /**
     * Sets the voltage of the intake motor.
     * 
     * @param voltage the voltage to set for the intake motor
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
     * Checks if a note is detected by the PE sensor.
     * 
     * @return true if a note is detected, false otherwise
     */
    public boolean noteDetected() {
        return !peSensor.get();
    }
}
