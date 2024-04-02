package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final DigitalInput peSensor = new DigitalInput(0);
    public static Intake intake = new Intake();

    private Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.burnFlash();
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public boolean noteDetected() {
        return !peSensor.get();
    }
}
