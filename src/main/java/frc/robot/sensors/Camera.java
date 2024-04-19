// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonVersion;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private long programStartTime = System.currentTimeMillis();

  // Gets initial instantiation of Cameras - TK
  public PhotonCamera april = aprilGetInstance();
  public PhotonCamera shape = shapeGetInstance();

  /******************************************************************************
   * Update the following variables to ensure that the cameras are instanciated *
   * properly!                                                                  *
   ******************************************************************************/
  private final String aprilTagCameraName = "april";
  private final String shapeCameraName = "shape";

  // Measurements to Cameras
  private Transform3d robotToApril = new Transform3d(new Translation3d(-Constants.botLength / 2, 0.0, 0.5),
      new Rotation3d(0, 0, Math.PI));

  // private Transform3d robotToNote = new Transform3d(new Translation3d(0.5, 0.0,
  // 0.5), new Rotation3d(0, 0, 0));

  /******************************************************************************
   * Update the following variables to ensure that the pose is calculated       *
   * properly!                                                                  *
   ******************************************************************************/
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonPoseEstimator aprilTagPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToApril);

  private boolean connected = false;
  private int connectionAttempts = 2;

  private static boolean versionMatches = false;

  // This thread allows this connection check to run in the background and not
  // block other Subsystems.
  private Thread attemptReconnection = null;

  // Time to delay periodic method Networktable connection check. IN
  // Seconds!!
  private double delayTime = 5;

  // Time to delay connection attempts is in SECONDS! - TK
  private double attemptDelay;

  // minAmbiguity is a percent expressed as a decimal - TK
  private double minAmbiguity;

  /**
   * Represents data related to an AprilTag detection.
   */
  public class aprilTagData {
    public final boolean isDetected;
    public final double distance;
    public final double ambiguity;
    public final double angle;
    public final int id;

    /**
     * Constructs an instance of the AprilTag data.
     *
     * @param isDetected true if an AprilTag is detected, false otherwise
     * @param dist       the distance to the AprilTag
     * @param ambiguity  the ambiguity of the AprilTag detection
     * @param angle      the angle to the AprilTag
     * @param id         the ID of the AprilTag
     */
    public aprilTagData(boolean isDetected, double dist, double ambiguity, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.ambiguity = ambiguity;
      this.angle = angle;
      this.id = id;
    }
  }

  /**
   * Represents data about a detected shape.
   */
  public class ShapeData {
    public final double area;
    public final double angle;

    /**
     * Constructs a new ShapeData object.
     * 
     * @param area  the distance to the detected shape
     * @param angle the angle to the detected shape
     */
    public ShapeData(double area, double angle) {
      this.area = area;
      this.angle = angle;
    }
  }

  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class AprilTagMeasurement {
    public final double Xdistance;
    public final double Ydistance;
    public final double yaw;
    public final double ambiguity;
    public final int id;

    /**
     * Constructs a new DistMeasurement object with the specified distance and
     * ambiguity values.
     * 
     * @param Xdistance the measured distance in the X axis
     * @param Ydistance the measured distance in the Y axis
     * @param yaw       the measured yaw
     * @param ambiguity the ambiguity of the distance measurement
     * @param id        the fiducial id of the detected apriltag
     */
    public AprilTagMeasurement(double Xdistance, double Ydistance, double yaw, double ambiguity, int id) {
      this.Xdistance = Xdistance;
      this.Ydistance = Ydistance;
      this.yaw = yaw;
      this.ambiguity = ambiguity;
      this.id = id;
    }
  }

  /**
   * Represents a camera used for vision processing.
   * This class handles the connection and configuration of the camera.
   *
   * @param PhotonvisionConnectionAttempts The number of connection attempts to
   *                                       make to PhotonVision.
   * @param delayBetweenAttempts           The delay in seconds between each
   *                                       connection attempt.
   * @param minAmbiguity                   The minimum ambiguity value for
   *                                       AprilTags.
   */
  private Camera(int PhotonvisionConnectionAttempts, double delayBetweenAttempts, double minAmbiguity) {
    attemptDelay = delayBetweenAttempts;

    this.minAmbiguity = minAmbiguity;

    aprilTagPoseEstimator.setReferencePose(new Pose2d(0, 0, new Rotation2d()));

    while (!connected && connectionAttempts <= PhotonvisionConnectionAttempts) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        versionMatches = checkVersion();
        aprilGetInstance();
        shapeGetInstance();
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Attempt: " + connectionAttempts + "\nChecking for PhotonVision connection in "
            + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
        connectionAttempts++;
      }
    }

    setNetworktableStatus();
  }

  /**
   * Represents a camera used for vision processing.
   * 
   * @return instance of Camera class
   */
  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(5, 1, 0.1);
    }
    return instance;
  }

  /**
   * This method attempts to get an instance of the PhotonCamera for AprilTag
   * detection. You must update the camera name variable at the top of the class
   * for this method to work. If an instance already exists, it returns that
   * instance. If not, it creates a new instance. Before creating a new instance,
   * it checks if the version of the library matches the expected version. It will
   * be reinstated if it loses connection.
   * 
   * @return The existing or newly created PhotonCamera instance for AprilTag
   *         detection.
   */
  private PhotonCamera aprilGetInstance() {
    checkVersion();
    if (versionMatches && april == null) {
      april = new PhotonCamera(inst, aprilTagCameraName);
    }
    return april;
  }

  /**
   * This method attempts to get an instance of the PhotonCamera for AprilTag
   * detection. You must update the camera name variable at the top of the class
   * for this method to work. If an instance already exists, it returns that
   * instance. If not, it creates a new instance. Before creating a new instance,
   * it checks if the version of the library matches the expected version. It will
   * be reinstated if it loses connection.
   * 
   * @return The existing or newly created PhotonCamera instance for shape
   *         detection.
   */
  private PhotonCamera shapeGetInstance() {
    checkVersion();
    if (versionMatches && shape == null) {
      shape = new PhotonCamera(inst, shapeCameraName);
    }
    return shape;
  }

  /**
   * Checks the version of the connected PhotonVision camera.
   * 
   * @return true if the version matches the expected version, false otherwise.
   */
  private boolean checkVersion() {
    Timer timeout = new Timer();
    String version = "";

    timeout.start();

    while (connected && version == "" && !timeout.hasElapsed(delayTime)) {
      version = inst.getTable("photonvision").getEntry("version").getString("");
      if (version == "") {
        System.err.println("Photon version not available yet...");
      }
      if (timeout.hasElapsed(delayTime)) {
        System.err.println("checkVersion loop Timedout for safety!");
      }
    }

    return PhotonVersion.versionMatches(version);
  }

  /**
   * Checks the connection status of the cameras.
   * If either camera is working or connected, sets the 'connected' flag to true.
   * Otherwise, sets the 'connected' flag to false.
   * 
   * @return the connection status of the cameras as boolean
   */
  private boolean testConnection() {
    // TODO: look into other alternative ways to check connection because there is
    // an issue where the .isConnected() method doesn't update even if the camera is
    // disconnected

    // If either camera is working || connected == true
    if (april.isConnected() || shape.isConnected()) {
      connected = true;
    } else {
      connected = false;
    }

    return connected;
  }

  /**
   * Attempts to reconnect to PhotonVision if the connection is lost.
   * This method continuously checks for a connection until it is successfully
   * established.
   */
  private void attemptToReconnect() {
    System.err.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");
    while (!connected) {
      if (testConnection()) {
        connected = true;
        versionMatches = checkVersion();
        aprilGetInstance();
        shapeGetInstance();
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in " + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
      }
    }
  }

  /**
   * Sets the status of the network table.
   * This method updates various entries in the network table to reflect the
   * current status of the camera and its connections.
   * If an error occurs while updating the network table, an error message is
   * printed to the console.
   */
  private void setNetworktableStatus() {
    try {
      NetworkTable status = inst.getTable("Vision").getSubTable("Status");

      status.getEntry("Version Matches: ").setBoolean(versionMatches);
      status.getSubTable("Version Info").getEntry("Photon Version: ")
          .setString(inst.getTable("photonvision").getEntry("version").getString("Version not available..."));
      status.getSubTable("Version Info").getEntry("Photon Lib Version: ")
          .setString(PhotonVersion.versionString);
      status.getEntry("Connection: ").setBoolean(connected);

      if (connected && versionMatches) {
        if (april != null) {
          status.getSubTable("Camera Status").getEntry("April Connection: ")
              .setBoolean(april.isConnected());
        }
        if (shape != null) {
          status.getSubTable("Camera Status").getEntry("Shape Connection: ")
              .setBoolean(shape.isConnected());
        }
      }
    } catch (Error e) {
      System.out.println("An error occured in Camera: \nUnable to publish status to Networktables:\n" + e);
    }
  }

  /**
   * This method is called periodically to perform camera-related tasks.
   * It checks if a certain amount of time has passed and if the camera connection
   * is still active. If necessary, it attempts to reconnect to the camera.
   * It also updates the network table information periodically.
   */
  @Override
  public void periodic() {
    try {
      // Was using Timer.delay() function here, but this caused issues with the other
      // subsystems...
      if ((System.currentTimeMillis() - programStartTime / 1000) % delayTime == 0 && attemptReconnection != null
          && !attemptReconnection.isAlive() && !testConnection()) {
        // Update Networktable information periodically - TK
        setNetworktableStatus();

        try {
          if (attemptReconnection == null || !attemptReconnection.isAlive()) {
            attemptReconnection = new Thread(() -> this.attemptToReconnect());

            attemptReconnection.start();
          }
        } catch (IllegalThreadStateException e) {
          System.out
              .println("Exception occured in Camera: \n" + e + "\nThread state: " + attemptReconnection.getState());
        }
      }
    } catch (Error e) {
      System.out.println("An error occured in Camera: \n" + e);
    }
  }

  /**
   * Gets the status of the camera.
   * 
   * @return true if the camera is connected and the version matches, false
   *         otherwise.
   */
  public boolean getStatus() {
    setNetworktableStatus();

    // Just returns the boolean that shows connction status - TK
    if (versionMatches && connected) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Checks if an AprilTag is detected by the camera.
   * 
   * @return true if an AprilTag is detected, false otherwise.
   */
  public boolean getAprilTagDetected() {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return true;
    }
    return false;
  }

  /**
   * Returns the ID of the detected AprilTag.
   * If the function returns null, it means that no targets were detected.
   *
   * @return The ID of the detected AprilTag, or null if no targets were detected.
   */
  public Integer getBestApriltagID() {
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getFiducialId();
    } else {
      return null;
    }
  }

  /**
   * If the camera is connected, version matches, and it has targets, this class
   * will compile all of the important apriltag information into one object.
   * 
   * @param xDist     the distance along the x axis
   * @param yDist     the distance along the y axis
   * @param yaw       the relative yaw to the target
   * @param ambiguity the ambiguity of the measurement
   * @param id        the fiducial id of the associated apriltag
   * 
   * @return ApriltagMeasurement object
   */
  public AprilTagMeasurement getBestAprilTagData() {
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    double xDist;
    double yDist;
    double yaw;
    double ambiguity;
    int id;

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      // Get X distance
      xDist = target.getBestCameraToTarget().getY();

      // Get Y distance
      yDist = target.getBestCameraToTarget().getX();

      // Get Yaw
      yaw = target.getYaw();

      // Get ambiguity of measurements
      ambiguity = target.getPoseAmbiguity();

      // Get fiducial ID of tracked Apriltag
      id = target.getFiducialId();

      return new AprilTagMeasurement(xDist, yDist, yaw, ambiguity, id);
    } else {
      return null;
    }
  }

  /**
   * If the camera is connected, version matches, and it has targets, this class
   * will compile all of the important apriltag information into one object.
   * 
   * @param xDist     the distance along the x axis
   * @param yDist     the distance along the y axis
   * @param yaw       the relative yaw to the target
   * @param ambiguity the ambiguity of the measurement
   * 
   * @return ApriltagMeasurement object
   */
  public AprilTagMeasurement getAprilTagData(int id) {
    double xDist;
    double yDist;
    double yaw;
    double ambiguity;

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          // Get X distance
          xDist = target.getBestCameraToTarget().getY();

          // Get Y distance
          yDist = target.getBestCameraToTarget().getX();

          // Get Yaw
          yaw = target.getYaw();

          // Get ambiguity of measurements
          ambiguity = target.getPoseAmbiguity();

          return new AprilTagMeasurement(xDist, yDist, yaw, ambiguity, target.getFiducialId());
        }
      }

      return null;
    } else {
      return null;
    }
  }

  /**
   * Returns the pitch angle of the detected AprilTag target.
   * If no targets are detected, it returns null.
   *
   * @return The pitch angle of the detected AprilTag target, or null if no
   *         targets
   *         are detected.
   */
  public Double getApriltagPitch() {
    // If this function returns a null, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getPitch();
    } else {
      return null;
    }
  }

  /**
   * Returns the pitch angle of the specified AprilTag target.
   * If no targets are detected, it returns null.
   *
   * @param id The ID of the AprilTag target.
   *
   * @return The pitch angle of the target, or null if no targets are detected.
   */
  public Double getApriltagPitch(int id) {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return target.getPitch();
        }
      }

      return null;
    } else {
      return null;
    }
  }

  /**************************************************************************************/

  /**
   * Calculates the distance to an AprilTag using the best available measurements.
   * If both the X and Y distances are available and the average ambiguity is
   * below the minimum ambiguity threshold,
   * the distance is calculated using the Pythagorean theorem.
   * If any of the measurements are missing or the average ambiguity is above the
   * threshold, the distance is considered 0.
   *
   * @return The calculated distance to the AprilTag.
   */
  public Double getBestAprilTagDist() {
    AprilTagMeasurement measurement = getBestAprilTagData();

    if (connected && versionMatches && measurement != null && measurement.ambiguity <= minAmbiguity) {
      return Math.sqrt((Math.pow(measurement.Xdistance, 2) + Math.pow(measurement.Ydistance, 2)));
    } else {
      return null;
    }
  }

  /**
   * Calculates the distance to an AprilTag based on its ID.
   * 
   * @param id The ID of the AprilTag.
   * 
   * @return The distance to the AprilTag, or 0 if the distance cannot be
   *         determined.
   */
  public Double getAprilTagDist(int id) {
    AprilTagMeasurement measurement = getAprilTagData(id);

    if (connected && versionMatches && measurement != null && measurement.ambiguity <= minAmbiguity) {
      return Math.sqrt((Math.pow(measurement.Xdistance, 2) + Math.pow(measurement.Ydistance, 2)));
    } else {
      return null;
    }
  }

  /**
   * Calculates the angle in degrees to the Apriltag.
   * 
   * @return The angle in degrees to the Apriltag. Returns null if the Apriltag is
   *         not detected or if the version does not match.
   */
  public Double getDegToApriltag() {
    AprilTagMeasurement measurement = getBestAprilTagData();

    if (connected && versionMatches && measurement != null && measurement.ambiguity <= minAmbiguity) {
      // Need to use the X distance for Y because we are calculating from the
      // Photonvision relative measurements - TK
      return Math.toDegrees(Math.atan2(measurement.Xdistance, measurement.Ydistance));
    } else {
      return null;
    }
  }

  /**
   * Calculates the angle in degrees to an Apriltag based on its ID.
   * 
   * @param id The ID of the Apriltag.
   * 
   * @return The angle in degrees to the Apriltag. Returns null if the Apriltag is
   *         not found or if the distance measurements are ambiguous. Returns null
   *         if the camera is not connected or the version does not match.
   */
  public Double getDegToApriltag(int id) {
    AprilTagMeasurement measurement = getAprilTagData(id);

    if (connected && versionMatches && measurement != null && measurement.ambiguity <= minAmbiguity) {
      // Need to use the X distance for Y because we are calculating from the
      // Photonvision relative measurements - TK
      return Math.toDegrees(Math.atan2(measurement.Xdistance, measurement.Ydistance));
    } else {
      return null;
    }
  }

  /**
   * Retrieves the layout of the AprilTag.
   *
   * @return the layout of the AprilTag
   */
  public AprilTagFieldLayout getAprilTagLayout() {
    return aprilTagFieldLayout;
  }

  /**
   * Returns whether a shape is detected by the camera.
   * 
   * @return true if a shape is detected, false otherwise.
   */
  public boolean getShapeDetected() {
    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets()) {
      return shape.getLatestResult().hasTargets();
    }
    return false;
  }

  /**
   * Returns the angle of the detected shape relative to the robot.
   * 
   * @return The angle of the shape relative to the robot, or null if no shape is
   *         detected.
   */
  public ShapeData getShapeData() {
    PhotonTrackedTarget target = shape.getLatestResult().getBestTarget();

    double yaw;
    double area;

    // Robot relative angle
    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets() && target != null) {
      // realative yaw to the shape
      yaw = target.getYaw();

      // area of the shape
      area = target.getArea();

      return new ShapeData(yaw, area);
    } else {
      return null;
    }
  }

  /**
   * Returns the current time in milliseconds since the program started.
   *
   * @return the current time in milliseconds
   */
  public double getCurrentTime() {
    // Shouldn't need to typecast here, but JIC :) - TK
    return (double) (System.currentTimeMillis() - programStartTime);
  }

  /**
   * Returns the latency in seconds for the camera feed.
   * 
   * @return The latency in seconds.
   */
  public double getLatencySeconds() {
    return april.getLatestResult().getLatencyMillis() / 1000;
  }

  /**
   * Returns an optional EstimatedRobotPose object representing the estimated
   * global pose of the robot.
   * The estimated global pose is obtained by setting the reference pose of the
   * AprilTagPoseEstimator
   * to the current pose of the SwerveDrive instance and updating it with the
   * latest result from the AprilTag.
   * 
   * @return an optional EstimatedRobotPose object representing the estimated
   *         global pose of the robot,
   *         or an empty optional if the pose cannot be estimated.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // Update this method accordingly to get the pose of the swerveDrive
    aprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose());
    return aprilTagPoseEstimator.update(april.getLatestResult());
  }
}