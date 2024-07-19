package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the top and bottom shooter flywheels, including their motors and
 * encoders.
 */
public class Shooter extends SubsystemBase {

    // Constants
    public static final double kShootSpeed = 0.9;
    public static final double kAmpSpeed = 0.3;
    public static final double kStopSpeed = 0.0;
    public static final double kPurgeSpeed = -0.3;

    // Singleton instance of the Shooter class
    public static Shooter instance = new Shooter();

    /**
     * The Shooter class represents the shooter subsystem of the robot.
     * It provides methods for controlling and managing the shooter mechanism.
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

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

    /**
     * The Shooter class represents the subsystem responsible for controlling the
     * shooter mechanism.
     * It initializes and configures the top and bottom shooter motors, encoders,
     * and other settings.
     */
    private Shooter() {
        topShooter.restoreFactoryDefaults();
        topShooter.setInverted(false);
        topShooter.setIdleMode(IdleMode.kCoast);
        topShooter.setInverted(false);
        topShooter.setSmartCurrentLimit(30);
        topShooter.burnFlash();
        topEncoder.setVelocityConversionFactor(1 / 42);

        bottomShooter.restoreFactoryDefaults();
        // bottomShooter.setInverted(true);
        bottomShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setInverted(false);
        bottomShooter.setSmartCurrentLimit(30);
        bottomShooter.burnFlash();
        bottomEncoder.setVelocityConversionFactor(1 / 42);
    }

    /**
     * This method is called periodically to update the values of the shooter speeds
     * on the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Speed", getTopShooterSpeed());
        SmartDashboard.putNumber("Bottom Speed", getBottomShooterSpeed());
    }

    /**
     * Sets the speed of the shooter.
     * 
     * @param speed the speed to set the shooter to
     */
    public void setShooterSpeed(double speed) {
        topShooter.set(speed);

    }

    /* */
    public void set(double speed) {
        topShooter.set(speed);
        bottomShooter.set(speed);
    }
    /**
     * Sets the voltage for both the top and bottom shooter motors.
     * 
     * @param voltage the voltage to set for the shooter motors
     */
    public void setShooterVoltage(double voltage) {
        topShooter.setVoltage(voltage);
        bottomShooter.setVoltage(voltage);
    }

    /**
     * Sets the voltage for the top shooter motor.
     * 
     * @param voltage the desired voltage for the top shooter motor
     */
    public void setShooterVoltageTop(double voltage) {
        topShooter.setVoltage(voltage + .15);
    }

    /**
     * Sets the voltage for the bottom shooter.
     * 
     * @param voltage the voltage to set for the bottom shooter
     */
    public void setShooterVoltageBottom(double voltage) {
        bottomShooter.setVoltage(voltage);
    }

    /**
     * Returns the speed of the top shooter.
     *
     * @return The speed of the top shooter.
     */
    public double getTopShooterSpeed() {
        return topEncoder.getVelocity();
    }

    /**
     * Returns the speed of the bottom shooter.
     *
     * @return the speed of the bottom shooter
     */
    public double getBottomShooterSpeed() {
        return bottomEncoder.getVelocity();
    }

    /**
     * Returns the speed of the shooter, which is the minimum value between the
     * bottom shooter speed and the top shooter speed.
     *
     * @return the speed of the shooter
     */
    public double getShooterSpeed() {
        return Math.min(getBottomShooterSpeed(), getTopShooterSpeed());
    }

}
