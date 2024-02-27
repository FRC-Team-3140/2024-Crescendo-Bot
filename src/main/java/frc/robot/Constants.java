package frc.robot;

import edu.wpi.first.math.util.Units;

public interface Constants {
    public static final double intakeVoltage = 7.5;
    public static final double gearRatio = 6.12;
    public static final int detectThreshold = 1000;
    public static final double botMass = 24.4;
    public static final double wheelDiameter = .1016;
    public static final double maxShootingDistance = 3.768725;
    public static final double botLength = Units.inchesToMeters(29);

    // In meters per second, determined from the free speed of the bot via
    // SwerveDriveSpecialties
    public static final double maxSpeed = 5.05968 * 1.4044;

    public static final double maxTurnSpeed = Double.MAX_VALUE; // These are basically infinite for our purposes
    public static final double maxAcceleration = 4000;
    public static final double botRadius = Math.hypot(botLength, botLength);
    // Max Speed divided by the circumference a circle determined by the distance of
    // the module from the center, divided by 2 pi to convert to radians
    public static final double maxChassisTurnSpeed = maxSpeed / botRadius;
    public double encoderRotationToMeters = 2 * Math.PI * ((wheelDiameter / 2) / gearRatio) / 42;
}
