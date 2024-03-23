package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static Shooter instance = new Shooter();

    public CANSparkMax topShooter = new CANSparkMax(9, MotorType.kBrushless);
    public RelativeEncoder topEncoder = topShooter.getEncoder();

    /**
     * Flywheel 2 on the shooter.
     */
    public CANSparkMax bottomShooter = new CANSparkMax(10, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 2's neo.
     */
    public RelativeEncoder bottomEncoder = bottomShooter.getEncoder();

    private Shooter() {
        topShooter.restoreFactoryDefaults();
        topShooter.setInverted(false);
        topShooter.setIdleMode(IdleMode.kCoast);
        topShooter.setInverted(false);
        topShooter.setSmartCurrentLimit(30);
        topShooter.burnFlash();
        topEncoder.setVelocityConversionFactor(1/42);

        bottomShooter.restoreFactoryDefaults();
        // bottomShooter.setInverted(true);
        bottomShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setInverted(true);
        bottomShooter.setSmartCurrentLimit(30);
        bottomShooter.burnFlash();
        bottomEncoder.setVelocityConversionFactor(1/42);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Speed", getTopShooterSpeed());        
        SmartDashboard.putNumber("Bottom Speed", getBottomShooterSpeed());
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void setShooterSpeed(double speed) {
        topShooter.set(speed);
    }

    public void setShooterVoltage(double voltage) {
        topShooter.setVoltage(voltage);
        bottomShooter.setVoltage(-voltage);
    }

    public void setShooterVoltageTop(double voltage) {
        topShooter.setVoltage(voltage + .15);
    }

    public void setShooterVoltageBottom(double voltage) {
        bottomShooter.setVoltage(-voltage);
    }

    public double getTopShooterSpeed() {
        return topEncoder.getVelocity();
    }

    public double getBottomShooterSpeed() {
        return bottomEncoder.getVelocity();
    }

    public double getShooterSpeed() {
        return Math.min(getBottomShooterSpeed(), getTopShooterSpeed());
    }

}
