package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the top and bottom shooter flywheels, including their motors and
 * encoders.
 */
public class Shooter extends SubsystemBase {

    public static Shooter instance = new Shooter();

    private static final double kShooterP = 0.008;
    private static final double kShooterD = 0.000;
    private static final double kShooterI = 0.000;

    public static final double kDistanceShootSpeed = 100.0;
    public static final double kCloseShootSpeed = 70.0;
    public static final double kAmpShootSpeed = 40.0;
    public static final double kAmpStopSpeed = 0.0;
    public static final double kAmpReverseSpeed = -1000.0;
    public static final double kSpeedDeadZone = 4.0;
    public static final double kSpeedTolerance = 100.0;
    public static final double kSetpointTimeout = 5.0;

    private static final int kShooterCurrentLimit = 30;
    private static final double kVelosityConversionFactor = 1.0 / 42.0;

    private double rpm_setpoint = 0.0;

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

    public PIDController topMotorPID = new PIDController(kShooterP, kShooterI, kShooterD);
    public PIDController bottomMotorPID = new PIDController(kShooterP, kShooterI, kShooterD);

    // input top bottom
    // 0.0 0 0
    // 0.1 10 12
    // 0.2 23 26
    // 0.3 34 39
    // 0.4 48 52
    // 0.5 61 65
    // 0.6 74 78
    // 0.7 87 91
    // 0.8 98 103
    // 0.9 110 115
    // 1.0 122 128
    // forward control splines for top and bottom
    InterpolatingDoubleTreeMap top_forward_est = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap bottom_forward_est = new InterpolatingDoubleTreeMap();

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
        topShooter.setSmartCurrentLimit(kShooterCurrentLimit);
        topEncoder.setVelocityConversionFactor(kVelosityConversionFactor);
        topShooter.burnFlash();

        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setInverted(false);
        bottomShooter.setSmartCurrentLimit(kShooterCurrentLimit);
        bottomEncoder.setVelocityConversionFactor(kVelosityConversionFactor);
        bottomShooter.burnFlash();

        // Map speed to input
        top_forward_est.put(0.0, 0.0);
        top_forward_est.put(10., 0.1);
        top_forward_est.put(23., 0.2);
        top_forward_est.put(34., 0.3);
        top_forward_est.put(48., 0.4);
        top_forward_est.put(61., 0.5);
        top_forward_est.put(74., 0.6);
        top_forward_est.put(87., 0.7);
        top_forward_est.put(98., 0.8);
        top_forward_est.put(110., 0.9);
        top_forward_est.put(122., 1.0);

        bottom_forward_est.put(0.0, 0.0);
        bottom_forward_est.put(12., 0.1);
        bottom_forward_est.put(26., 0.2);
        bottom_forward_est.put(39., 0.3);
        bottom_forward_est.put(52., 0.4);
        bottom_forward_est.put(65., 0.5);
        bottom_forward_est.put(78., 0.6);
        bottom_forward_est.put(91., 0.7);
        bottom_forward_est.put(103., 0.8);
        bottom_forward_est.put(115., 0.9);
        bottom_forward_est.put(128., 1.0);
    }

    /**
     * This method is called periodically to update the values of the shooter speeds
     * on the SmartDashboard.
     */
    @Override
    public void periodic() {

        double current_bottom_rpm = getBottomShooterSpeed();
        double current_top_rpm = getTopShooterSpeed();

        SmartDashboard.putNumber("Top Speed", current_bottom_rpm);
        SmartDashboard.putNumber("Bottom Speed", current_top_rpm);
        SmartDashboard.putNumber("Shooter Setpoint", rpm_setpoint);

        // cutoff motor if it has been running for too long without a setpoint update
        // if ((rpm_setpoint != 0.0) && (System.currentTimeMillis() * 0.001 - starttime
        // > kSetpointTimeout)) {
        // rpm_setpoint = 0.0;
        // }

        if (Math.abs(rpm_setpoint) < kSpeedDeadZone) {
            // if the setpoint is near zero, stop the motors
            topShooter.set(0.0);
            bottomShooter.set(0.0);
        } else {
            // System.out.printf("Shooter RPM: %.1f %.1f - %.1f\n", current_top_rpm,
            // current_bottom_rpm, rpm_setpoint);

            //

            // Apply a filter to the encoder speed to smooth it out
            // current_top_rpm = topFilter.calculate(current_top_rpm);
            // current_bottom_rpm = bottomFilter.calculate(current_bottom_rpm);

            // Read the velocity conversion factor from the encoder and print it out for
            // each motor
            // System.out.printf("Top Encoder Conversion Factor: %.3f\n",
            // topEncoder.getVelocityConversionFactor());
            // System.out.printf("Bottom Encoder Conversion Factor: %.3f\n",
            // bottomEncoder.getVelocityConversionFactor());

            // System.out.printf("Shooter RPM: %.1f %.1f - %.1f\n", current_top_rpm,
            // current_bottom_rpm, rpm_setpoint);

            double fw_top_motor_speed = top_forward_est.get(rpm_setpoint);
            double fw_bottom_motor_speed = bottom_forward_est.get(rpm_setpoint);

            double top_motor_speed = fw_top_motor_speed + topMotorPID.calculate(current_top_rpm, rpm_setpoint);
            double bottom_motor_speed = fw_bottom_motor_speed
                    + bottomMotorPID.calculate(current_bottom_rpm, rpm_setpoint);

            // top_motor_speed = 0.2;
            // bottom_motor_speed = 0.2;

            // input top bottom
            // 0.0 0 0
            // 0.1 10 12
            // 0.2 23 26
            // 0.3 34 39
            // 0.4 48 52
            // 0.5 61 65
            // 0.6 74 78
            // 0.7 87 91
            // 0.8 98 103
            // 0.9 110 115
            // 1.0 122 128

            // System.out.printf("Shooter RPM: T: %.1f B: %.1f - S: %.1f %.2f %.2f\n",
            // current_top_rpm, current_bottom_rpm,
            // rpm_setpoint,top_motor_speed,bottom_motor_speed);

            topShooter.set(top_motor_speed);
            bottomShooter.set(bottom_motor_speed);
        }
    }

    /**
     * Sets the RPM (Revolutions Per Minute) for the shooter mechanism.
     *
     * @param rpm The desired RPM to set for the shooter.
     * @return True if the RPM is at the setpoint.
     */
    public boolean setShooterRpm(double rpm) {

        rpm_setpoint = rpm;

        return atSetpoint();
    }

    public boolean atSetpoint() {
        return Math.abs(getShooterSpeed() - rpm_setpoint) < kSpeedTolerance;
    }

    /**
     * Stops the shooter by setting its RPM to a predefined stop speed.
     */
    public void stop() {
        this.setShooterRpm(kAmpStopSpeed);
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
