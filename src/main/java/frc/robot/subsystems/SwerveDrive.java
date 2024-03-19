// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Camera; // TODO: At this point camera measurements seem to be unreliable.  Needing this as part of the drivetrain which needs to be our most reliable subsystem is a big red flag and caused problems at smr.  We should consider moving camera dependencies outside of the subsystem.  Camera should be independed and integartion should probably be done at a "Command" level.
import frc.robot.sensors.Camera.DistAmb;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase implements Constants {

  private static SwerveDrive instance = new SwerveDrive();
  ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, .1, new Constraints(360, 720));
  SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  private final SwerveDrivePoseEstimator poseEstimator;
  private Camera camera = Camera.getInstance(); // TODO: How integrated is this????
  public static AHRS gyro; // TODO: This should be private and not static.  Is this needed elsewhere?

  private final Translation2d[] locations = {
      new Translation2d(botLength, botLength),
      new Translation2d(botLength, -botLength),
      new Translation2d(-botLength, botLength),
      new Translation2d(-botLength, -botLength)
  };

  SwerveModule[] modules = {
      new SwerveModule("frontLeft", 3, 8, 7, 0.701239),
      new SwerveModule("frontRight", 2, 6, 5, 0.707867),
      new SwerveModule("backLeft", 0, 2, 1, 0.219279),
      new SwerveModule("backRight", 1, 4, 3, 0.447409),

  };
  double odometryOffset = 0;

  private ChassisSpeeds botSpeeds = new ChassisSpeeds(0, 0, 0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      locations[0], locations[1], locations[2], locations[3]);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  // TODO: See if the following boolean is neccessary
  public boolean allowPathMirroring = false;

  /**
   * Constructs a new SwerveDrive.
   */
  public SwerveDrive() {
    NetworkTableInstance.getDefault().getTable("VisionStdDev").getEntry("VisionstdDev").setDouble(.01);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.PI / 45); // 4 degrees
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    // TODO: Should be last.
    // Autobuilder for Pathplanner Goes last in constructor! TK
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                         // Constants class
            new PIDConstants(4, 0.0, 0), // Translation PID constants
            new PIDConstants(4, 0.0, 0), // Rotation PID constants
            maxChassisSpeed, // Max module speed, in m/s
            botRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this::shouldFlipPath,
        this // Reference to this subsystem to set requirements
    );

    // TODO: I am still concerned about having vision measurements in the drivetrain. I would recommend you use very simple odomentry here and create a completly seperate odometer is a sepreate subsystem that interates with the camera.
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
        VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(30)));

    Logger.recordOutput("Actual States", states);
    Logger.recordOutput("Set States", swerveModuleStates);
    Logger.recordOutput("Odometry", poseEstimator.getEstimatedPosition());
  }

  // TODO: Dead code.  Remove it.
  // var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent() && allowPathMirroring) {
  // return alliance.get() == DriverStation.Alliance.Red;
  // }
  // return false;

  // TODO: should be at the start of the class.
  // WPILib
  StructArrayPublisher<SwerveModuleState> actualStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Actual States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> setStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Set States", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> odometryStruct = NetworkTableInstance.getDefault()
      .getStructTopic("Odometry", Pose2d.struct).publish();
  SwerveModuleState[] states = new SwerveModuleState[4];

 
  /** TODO: Double check that this comment is correct.
   * The method that is called periodically. This method is used to update the
   * odometry and the actual and set states of the swerve modules.
   * 
   * The method performs the following operations:
   * 
   * 1. Iterates over each swerve module and retrieves its current state, storing
   *    these states in the 'states' array.
   * 
   * 2. Calls the 'updateOdometry' method to update the robot's position based on
   *    the current states of the swerve modules.
   * 
   * 3. Sets the 'actualStates' and 'setStates' with the current and desired states
   *    of the swerve modules, respectively.
   * 
   * 4. Retrieves the standard deviation of the vision system from a network table
   *    and sets it in the 'poseEstimator'. This value is used to quantify the 
   *    uncertainty in the vision measurements. TODO: How likely is this to crash the drivetrain?
   * 
   * 5. Updates the 'odometryStruct' with the current pose of the robot.
   */
  public void periodic() {

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    updateOdometry();
    actualStates.set(swerveModuleStates);
    setStates.set(states);
    double visionStdDev = NetworkTableInstance.getDefault().getTable("VisionStdDev").getEntry("VisionstdDev")
        .getDouble(.02);
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDev, visionStdDev, Units.degreesToRadians(30)));
    // poseEstimator.addVisionMeasurement(camera.getEstimatedGlobalPose(),
    // camera.getTimestamp());
    odometryStruct.set(getPose());

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    botSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), .02);
    swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxModuleSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setStates(swerveModuleStates[i], false);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speeds A SwerveDriveSpeeds object that contains the desired speeds 
   *               for each of the robot's swerve modules. These speeds are 
   *               typically calculated based on joystick inputs.
   *
   * This method uses the provided speeds to control the movement of the robot. 
   * It sets the desired speed of each swerve module according to the corresponding 
   * speed in the SwerveDriveSpeeds object. The method ensures that the robot 
   * moves in the desired direction and at the desired speed based on the joystick inputs.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  /**
   * Method to drive the robot at a specific angle.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param angleRadians  The angle at which the robot should move.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveRobotAtAngle(double xSpeed, double ySpeed, double angleRadians, boolean fieldRelative) {
    thetaController.setGoal(angleRadians);
    double angularSpeed = thetaController.calculate(getPose().getRotation().getRadians());
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative);
  }

  /**
   * Method to drive the robot facing the speaker.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveRobotFacingSpeaker(double xSpeed, double ySpeed, boolean fieldRelative) {
    // TODO: this sort of logic may be too much for the drivetrain.  Consider moving it to a command.  That way if one command has issues it does not crash at the drivetrain level.
    Translation2d distanceFromSpeaker;
    if (shouldFlipPath()) {
      distanceFromSpeaker = getExpectedPose().getTranslation().minus(redSpeakerTranslation);
    } else {
      distanceFromSpeaker = getExpectedPose().getTranslation().minus(blueSpeakerTranslation);
    }
    double angle = Math.atan2(distanceFromSpeaker.getY(), distanceFromSpeaker.getX());

    driveRobotAtAngle(xSpeed, ySpeed, angle, fieldRelative);
  }


  /**
   * Updates the odometry of the swerve drive based on sensor inputs and vision measurements.
   * This method updates the pose estimator using the gyro rotation and the positions of the swerve modules.
   * It also incorporates vision measurements if the camera is connected and a valid pose and distance reading are available.
   * The vision measurements are added to the pose estimator with a timestamp.
   */
  public void updateOdometry() {
    poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getSwerveModulePosition(),
            modules[1].getSwerveModulePosition(),
            modules[2].getSwerveModulePosition(),
            modules[3].getSwerveModulePosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.

    // TODO: Remove dead code.  This is a lot of logic that can crash in the middle of the drive train.
    try {
      if (Camera.getInstance().isConnected()) {
        Optional<EstimatedRobotPose> pose = Camera.getInstance().getEstimatedGlobalPose();
        DistAmb reading = Camera.getInstance().getApriltagDistX();
        if (pose.isPresent() && reading != null && reading.ambiguity < 0.3
        /*
         * && getPose().getTranslation().getDistance(Camera.getInstance().
         * getEstimatedGlobalPose().get().estimatedPose
         * .getTranslation().toTranslation2d()) < .58
         */) {
          poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(),
              Timer.getFPGATimestamp());
          // System.out.println("Target Detected");
        }//  else {
        //    poseEstimator.addVisionMeasurement(getPose(), Timer.getFPGATimestamp());
        // }
      }
    } catch (Error test) {
      System.err.println(test);
    }
  }



  /**
   * Returns an array of SwerveModulePosition objects representing the positions of the swerve modules.
   *
   * @return an array of SwerveModulePosition objects
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  /** TODO: How is this different that the previous.  Are they both needed?
   * Returns an array of SwerveModuleState objects representing the current state of each swerve module.
   *
   * @return an array of SwerveModuleState objects
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  /** TODO: I think this is logic that is out of place in the drivetrain.  Why is it here? Move to robot container?
   * Determines whether the path should be flipped based on the current alliance.
   * 
   * @return true if the path should be flipped for the Red alliance, false otherwise.
   */
  public boolean shouldFlipPath() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  /**
   * Returns the robot-relative speeds of the chassis.
   *
   * @return The robot-relative speeds of the chassis.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return the current pose of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** TODO: The need for this seems questionable.  I think you may be doing something wrong.
   * Returns the expected pose of the robot.
   * The expected pose is calculated by adding a transformation to the current pose.
   * The transformation is determined by the robot's velocity in the x and y directions.
   * 
   * @return the expected pose of the robot
   */
  public Pose2d getExpectedPose() {
    return getPose().plus(new Transform2d(
        new Translation2d(botSpeeds.vxMetersPerSecond * .05, botSpeeds.vyMetersPerSecond * .05), new Rotation2d()));
  }

  /** TODO: Logic not needed in drivetrain.  Needs to be somewhere else.  Probably Commands.
   * Calculates the expected distance from the speaker based on the current pose.
   * If the path should be flipped, it uses the red speaker translation, otherwise it uses the blue speaker translation.
   * 
   * @return The expected distance from the speaker.
   */
  public double getExpectedDistanceFromSpeaker() {
    // .05 is a placeholder timesteo, may be changed in the future
    if (shouldFlipPath()) {
      return redSpeakerTranslation.getDistance(getExpectedPose().getTranslation());
    }
    return blueSpeakerTranslation.getDistance(getExpectedPose().getTranslation());
  }

  /** TODO: Again why two functions that do the same thing.  You are doing something wrong.
   * Returns the distance from the speaker based on the current pose.
   * If the path should be flipped, the distance is calculated using the red speaker translation.
   * Otherwise, the distance is calculated using the blue speaker translation.
   *
   * @return the distance from the speaker
   */
  public double getDistanceFromSpeaker() {
    if (shouldFlipPath()) {
      return redSpeakerTranslation.getDistance(getPose().getTranslation());
    }
    return blueSpeakerTranslation.getDistance(getPose().getTranslation());
  }

  /**
   * Return the instance of the SwerveDrive subsystem.
   * 
   * @return the instance of the SwerveDrive subsystem.
   */
  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive();
    }
    return instance;
  }

  /** TODO: Bad idea to have this here.
   * Sets the standard deviation of the vision system.
   * 
   * @param deviation The standard deviation of the vision system.
   */
  public void setVisionStdDeviations(double deviation) {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(deviation, deviation, Units.degreesToRadians(30)));
  }

  // TODO: Should be at the start of the class.
  public PIDController turnPID = new PIDController(.5, 0.0, 0);

  // TODO: Should be in camera or command.  Not in drivetrain.
  public double turnToAprilTag(int ID) {
    // TODO: Potential null error unhandled here 
    // turnPID.enableContinuousInput(0, 360);
    double botAngle = getPose().getRotation().getDegrees();
    double offsetAngle = camera.getDegToApriltag(ID);
    double setpoint = 0;
    if (botAngle - offsetAngle <= 0)
      setpoint = botAngle + offsetAngle;
    else
      setpoint = botAngle - offsetAngle;

    turnPID.setSetpoint(setpoint);
    return turnPID.calculate(botAngle);
  }

  /**
   * Resets the gyro.
   */
  public void resetGyro() {
    odometryOffset = gyro.getAngle();
    gyro.reset();
  }

  /** TODO: Based on path planner documentation, Does pathplanner update the position?  Does this conflict with the camera updating the position?
   * Resets the pose of the robot.
   * 
   * @param pose The new pose of the robot.
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

}