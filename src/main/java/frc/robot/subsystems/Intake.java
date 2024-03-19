package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* 
 * TODO: Apply the following improvements:
 * 
 * 1. Singleton Pattern: Correctly implement the Singleton pattern. Initialize 
 * 'intake' as null to allow new instance creation in 'getInstance()'.
 *
 * 2. Access Modifiers: Make 'intakeMotor' and 'intake' variables private. 
 * Provide public getter methods for controlled access.
 *
 * 3. Constants: Define motor ID and sensor port as constants for easier 
 * management and modification.
 */


/**
 * The Intake class represents the intake subsystem of the robot.
 * It controls the intake motor and its associated settings.
 */
public class Intake extends SubsystemBase {
    public CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final DigitalInput peSensor = new DigitalInput(0);
    public static Intake intake = new Intake();
    
    /**
     * The Intake class represents the intake subsystem of the robot.
     * It controls the intake motor and its associated settings.
     */
    private Intake(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(30); // TODO: This seems way to high.  Test with lower values.
        intakeMotor.burnFlash();
    }
    
    /**
     * Returns the instance of the Intake class.
     * 
     * @return intake
     */
    public static Intake getInstance(){
        if(intake == null){
            intake = new Intake();
        }
        return intake;
    }
    
    /**
     * Sets the intake motor to a given voltage.
     * 
     * @param voltage
     */
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    /**
     * Stops the intake motor.
     */
    public boolean noteDetected(){
        return !peSensor.get();
    }
}
