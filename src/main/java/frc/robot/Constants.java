package frc.robot;

public interface Constants {
    public static final double gearRatio = 6.12;
    public static final double botMass = 24.4;
    public static final double wheelDiameter = .1016;
    public static final double botLength = .254;
    public static final double botWidth = .254;
    public static final double maxSpeed = 5.05968; // In meters per second, determined from the free speed of the bot via SwerveDriveSpecialties
    public static final double maxTurnSpeed = Double.MAX_VALUE; //These are basically infinite for our purposes 
    public static final double maxAcceleration = Double.MAX_VALUE;
    public static final double maxChassisTurnSpeed = maxSpeed/Math.hypot(botLength, botWidth); // Max Speed divided by the circumference a circle determined by the distance of the module from the center, divided by 2 pi to convert to radians
    public double conversionFactor = 1.0/19.85; 
}