// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.ExampleGlobalMeasurementSensor;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive implements Constants{
  private final Translation2d[] locations = {
    new Translation2d(botLength, botWidth),
    new Translation2d(botLength, -botWidth),
    new Translation2d(-botLength, botWidth),
    new Translation2d(-botLength, -botWidth)
  };

  SwerveModule[] modules = {
    new SwerveModule("frontLeft", 0, 2, 1, 0.969279),
    new SwerveModule("frontRight", 1, 4, 3, 0.697409),
    new SwerveModule("backLeft", 2, 6, 5, 0.707867),
    new SwerveModule("backRight", 3, 8, 7,  0.701239),
  };

  private static AHRS gyro = RobotContainer.gyro;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          locations[0],locations[1],locations[2],locations[3]);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public SwerveDrive() {
    gyro.reset();
    // AutoBuilder.configureHolonomic(
    //             this::getPose, // Robot pose supplier
    //             this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
    //                                              // Constants class
    //                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                     new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                     4.5, // Max module speed, in m/s
    //                     0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig() // Default path replanning config. See the API for the options here
    //             ),
    //             this // Reference to this subsystem to set requirements
    //     );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether  the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    
    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i], false);
    }
  }
  

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          modules[0].getSwerveModulePosition(),
          modules[1].getSwerveModulePosition(),
          modules[2].getSwerveModulePosition(),
          modules[3].getSwerveModulePosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }
}
