package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {    private static NavX instance;
    private AHRS navx;

    // Private constructor to prevent instantiation
    private NavX() {
        navx = new AHRS(SPI.Port.kMXP);
    }

    // Public method to provide access to the singleton instance
    public static NavX getInstance() {
        if (instance == null) {
            instance = new NavX();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add code here to update any necessary state or perform periodic checks
        System.out.printf("NavX Yaw: %.2f\n",getYaw());
        System.out.printf("NavX Calibrating: %b\n",isCalibrating());
        System.out.printf("NavX Compass Heading: %.2f\n",getCompassHeading());
    }

    // Example method to get the current yaw angle
    public double getYaw() {
        return navx.getYaw();
    }

    public double getCompassHeading() {
        return navx.getCompassHeading();
    }   

    // Example method to reset the yaw angle
    public void resetYaw() {
        navx.zeroYaw();
    }

    public boolean isCalibrating() {
        return navx.isCalibrating();
    }
}