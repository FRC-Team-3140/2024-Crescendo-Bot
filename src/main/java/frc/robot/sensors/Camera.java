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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
  public PhotonCamera shape = notesGetInstance();

  // Measurements to Cameras
  private Transform3d robotToApril = new Transform3d(new Translation3d(-Constants.botLength / 2, 0.0, 0.5),
      new Rotation3d(0, 0, Math.PI));

  // TODO: get measurements for note cam - TK : private Transform3d robotToNote =
  // new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

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

  // Global fiducial ids for important landmark apriltags - TK
  public aprilTagLayout aprilTagLayout;
  private int speakerAprilTagC;
  private int ampAprilTag;
  private int sourceAprilTag;


  /**
   * Represents the layout of AprilTags in the camera's field of view.
   */
  public class aprilTagLayout {
    public final int speakerAprilTag;
    public final int ampAprilTag;
    public final int sourceAprilTag;

    /**
     * Constructs a new instance of the AprilTag layout.
     * 
     * @param speaker The AprilTag ID for the speaker.
     * @param amp The AprilTag ID for the amplifier.
     * @param source The AprilTag ID for the audio source.
     */
    public aprilTagLayout(int speaker, int amp, int source) {
      this.speakerAprilTag = speaker;
      this.ampAprilTag = amp;
      this.sourceAprilTag = source;
    }
  }

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
     * @param isDetected  true if an AprilTag is detected, false otherwise
     * @param dist        the distance to the AprilTag
     * @param ambiguity   the ambiguity of the AprilTag detection
     * @param angle       the angle to the AprilTag
     * @param id          the ID of the AprilTag
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
      public final boolean isDetected;
      public final double distance;
      public final double angle;

      /**
       * Constructs a new ShapeData object.
       * 
       * @param isDetected true if a shape is detected, false otherwise
       * @param distance the distance to the detected shape
       * @param angle the angle to the detected shape
       */
      public ShapeData(boolean isDetected, double distance, double angle) {
        this.isDetected = isDetected;
        this.distance = distance;
        this.angle = angle;
      }
    }

  /**
   * Represents a distance measurement obtained from a camera sensor.
   */
  public class Measurement {
    public final double distance;
    public final double angle;
    public final double ambiguity;

    /**
     * Constructs a new DistMeasurement object with the specified distance and ambiguity values.
     * 
     * @param distance   the measured distance
     * @param ambiguity  the ambiguity of the distance measurement
     */
    public Measurement(double distance, double angle, double ambiguity) {
      this.distance = distance;
      this.angle = angle;
      this.ambiguity = ambiguity;
    }
  }

  /**
   * Represents a camera used for vision processing.
   * This class handles the connection and configuration of the camera.
   *
   * @param PhotonvisionConnectionAttempts The number of connection attempts to make to PhotonVision.
   * @param delayBetweenAttempts The delay in seconds between each connection attempt.
   * @param minAmbiguity The minimum ambiguity value for AprilTags.
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
        notesGetInstance();
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

    configureTeam();
  }

  /**
   * Represents a camera used for vision processing.
   */
  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(5, 1, 0.1);
    }
    return instance;
  }

  /**
   * This method attempts to get an instance of the PhotonCamera for AprilTag detection.
   * If an instance already exists, it returns that instance. If not, it creates a new instance.
   * Before creating a new instance, it checks if the version of the library matches the expected version.
   * It will be reinstated if it loses connection.
   * 
   * @return The existing or newly created PhotonCamera instance for AprilTag detection.
   */
  private PhotonCamera aprilGetInstance() {
    checkVersion();
    if (versionMatches && april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  /**
   * This method attempts to get an instance of the PhotonCamera for shape detection.
   * If an instance already exists, it returns that instance. If not, it creates a new instance.
   * Before creating a new instance, it checks if the version of the library matches the expected version.
   * It will be reinstated if it loses connection.
   * 
   * @return The existing or newly created PhotonCamera instance for shape detection.
   */
  private PhotonCamera notesGetInstance() {
    checkVersion();
    if (versionMatches && shape == null) {
      shape = new PhotonCamera(inst, "shape");
    }
    return shape;
  }

  /**
   * Configures the team-specific settings for the camera.
   * If the alliance is Red, it sets the apriltag constant id's for the left side.
   * If the alliance is not Red, it sets the apriltag constant id's for the right side.
   */
  private void configureTeam() {
    // Configures apriltag constant id's
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      speakerAprilTagC = 4; // Apriltag on left
      ampAprilTag = 5;
      sourceAprilTag = 10; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTagC, ampAprilTag, sourceAprilTag);
    } else {
      speakerAprilTagC = 7; // Apriltag on left
      ampAprilTag = 6;
      sourceAprilTag = 2; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTagC, ampAprilTag, sourceAprilTag);
    }
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
   * @return the connection status of the cameras
   */
  private boolean testConnection() {
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
   * This method continuously checks for a connection until it is successfully established.
   */
  private void attemptToReconnect() {
    System.err.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");
    while (!connected) {
      if (testConnection()) {
        connected = true;
        versionMatches = checkVersion();
        aprilGetInstance();
        notesGetInstance();
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
   * This method updates various entries in the network table to reflect the current status of the camera and its connections.
   * If an error occurs while updating the network table, an error message is printed to the console.
   */
  private void setNetworktableStatus() {
    try {
      // TODO: Simplify this repeated code: inst.getTable("Vision").getSubTable("Status")
      inst.getTable("Vision").getSubTable("Status").getEntry("Version Matches: ").setBoolean(versionMatches);
      inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Version: ")
          .setString(inst.getTable("photonvision").getEntry("version").getString("Version not available..."));
      inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Lib Version: ")
          .setString(PhotonVersion.versionString);
      inst.getTable("Vision").getSubTable("Status").getEntry("Connection: ").setBoolean(connected);

      if (connected && versionMatches && april != null) {
        inst.getTable("Vision").getSubTable("Status").getSubTable("Camera Status").getEntry("April Connection: ")
            .setBoolean(april.isConnected());
        inst.getTable("Vision").getSubTable("Status").getSubTable("Camera Status").getEntry("Notes Connection: ")
            .setBoolean(shape.isConnected());
      }
    } catch (Error e) {
      System.out.println("An error occured in Camera: \nUnable to publish status to Networktables:\n" + e);
    }
  }

  /**
   * This method is called periodically to perform camera-related tasks.
   * It checks if a certain amount of time has passed and if the camera connection is still active.
   * If necessary, it attempts to reconnect to the camera.
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

      // TODO: Remove this debugging code
      // Debugging Networktable Entries
      // aprilTagLocation tag = getAprilTagLocation(speakerAprilTag);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("ID:
      // ").setInteger(tag.id);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Detected:
      // ").setBoolean(tag.isDetected);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Dist:
      // ").setDouble(tag.distance);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Ambiguity").setDouble(tag.ambiguity);
      // inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees:
      // ").setDouble(tag.angle);
    } catch (Error e) {
      System.out.println("An error occured in Camera: \n" + e);
    }
  }

  /**
   * Gets the status of the camera.
   * 
   * @return true if the camera is connected and the version matches, false otherwise.
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
   * Retrieves the latest AprilTag detection with the specified tag ID.
   * 
   * @param tag_id the ID of the AprilTag to search for
   * @return the PhotonTrackedTarget object representing the detected AprilTag, or null if not found
   */
  public PhotonTrackedTarget latestAprilTagDetection(int tag_id) {
    // Check if hasTargets is true, then check if the tag_id is equal to the
    if (april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == tag_id) {
          System.out.println("Tag ID: " + target.getFiducialId() + " X: " + target.getBestCameraToTarget().getX()
              + " Y: " + target.getBestCameraToTarget().getY());
          return target;
        }
      }
    }

    return null;
  }

  /**
   * Checks if an AprilTag is detected by the camera.
   * 
   * @return true if an AprilTag is detected, false otherwise.
   */
  public boolean getAprilTagDetected() {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().hasTargets();
    }
    return false;
  }

  /**
   * Returns the ID of the detected AprilTag.
   * If the function returns 0, it means that no targets were detected.
   *
   * @return The ID of the detected AprilTag, or -1 if no targets were detected.
   */
  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getFiducialId();
    } else {
      return -1;
    }
  }

  /**
   * Returns the yaw angle of the detected AprilTag target.
   * If no targets are detected, it returns 999. TODO:  codes are not a good way to do this. 
   *
   * @return The yaw angle of the detected AprilTag target, or 999 if no targets are detected.
   */
  public double getApriltagYaw() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getYaw();
    } else {
      return 999;
    }
  }

  /**
   * Returns the yaw angle of the specified Apriltag target.
   * If the function returns 999, it means there are no detected targets.
   *
   * @param id The ID of the Apriltag target.
   * @return The yaw angle of the specified Apriltag target, or 999 if no targets are detected.
   */
  public double getApriltagYaw(int id) {
    // If this function returns a 999, that means there is not any detected targets
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return target.getYaw();
        }
      }
      return 0;
    } else {
      return 999;
    }
  }

  /**
   * Returns the pitch angle of the detected AprilTag target.
   * If no targets are detected, it returns 999.
   *
   * @return The pitch angle of the detected AprilTag target, or 999 if no targets are detected.
   */
  public double getApriltagPitch() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getPitch();
    } else {
      return 999;
    }
  }

  /**
   * Returns the pitch angle of the specified AprilTag target.
   * If no targets are detected, it returns 999.
   *
   * @param id The ID of the AprilTag target.
   * @return The pitch angle of the target, or 999 if no targets are detected.
   */
  public double getApriltagPitch(int id) {
    // If this function returns a 999, that means there is not any detected targets
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return target.getPitch();
        }
      }
      return 0;
    } else {
      return 999;
    }
  }

  /**
   * Represents a distance measurement from the camera to the best (largest) target.
   * The distance is measured along the X-axis.
   * The pose ambiguity indicates the uncertainty in the target's pose estimation.
   * 
   * @return The distance measurement from the camera to the best target, or null if no targets are detected or the camera is not connected.
   */
  public Measurement getBestApriltagDistX() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new Measurement(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }
  
  /**
   * Retrieves the distance measurement for a specific AprilTag ID.
   *
   * @param id The ID of the AprilTag.
   * @return The distance measurement for the specified AprilTag ID, or null if the tag is not found or the camera is not connected.
   */
  public Measurement getApriltagDistX(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new Measurement(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  /**
   * Represents a distance measurement from the camera to the best (largest) target.
   * 
   * @return The distance measurement from the camera to the best target, or 0 if no targets are detected.
   */
  public Measurement getBestApriltagDistY() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new Measurement(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  /**
   * Represents a distance measurement from the camera to a specific target id.
   * 
   * @param id The ID of the target.
   * @return The distance measurement from the camera to the specified target ID, or null if the tag is not found or the camera is not connected.
   */
  public Measurement getApriltagDistY(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new Measurement(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
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
   * If both the X and Y distances are available and the average ambiguity is below the minimum ambiguity threshold,
   * the distance is calculated using the Pythagorean theorem.
   * If any of the measurements are missing or the average ambiguity is above the threshold, the distance is considered 0.
   *
   * @return The calculated distance to the AprilTag.
   */
  public double getBestAprilTagDist() {
    Measurement distX = getBestApriltagDistX();
    Measurement distY = getBestApriltagDistY();

    if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
      return Math.sqrt((Math.pow(distX.distance, 2) + Math.pow(distY.distance, 2)));
    } else {
      return 0;
    }
  }

  /**
   * Calculates the distance to an AprilTag based on its ID.
   * 
   * @param id The ID of the AprilTag.
   * @return The distance to the AprilTag, or 0 if the distance cannot be determined.
   */
  public double getAprilTagDist(int id) {
    Measurement distX = getApriltagDistX(id);
    Measurement distY = getApriltagDistY(id);

    if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
      return Math.sqrt((Math.pow(distX.distance, 2) + Math.pow(distY.distance, 2)));
    } else {
      return 0;
    }
  }

  /**
   * Calculates the angle in degrees to the Apriltag.
   * 
   * @return The angle in degrees to the Apriltag. Returns 999 if the Apriltag is not detected or if the version does not match.
   */
  public double getDegToApriltag() {
    Measurement distX = getBestApriltagDistX();
    Measurement distY = getBestApriltagDistY();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      /*
       * Takes Photonvision Z angle theta value (3D processing mode on camera) and
       * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
       * left the # of degs. that arcTan or inverse tan returns from the X & Y
       * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
       * coordinates. - TK
       */

      // Need to use the getX method that we wrote for Y in atan because it returns
      // the Photon Y. - TK
      if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
        return Math.toDegrees(Math.atan2(distX.distance, distY.distance));
      } else {
        return 999;
      }
    } else {
      return 999;
    }
  }

  /**
   * Calculates the angle in degrees to an Apriltag based on its ID.
   * 
   * @param id The ID of the Apriltag.
   * @return The angle in degrees to the Apriltag. Returns 999 if the Apriltag is not found or if the distance measurements are ambiguous. Returns 9999 if the camera is not connected or the version does not match.
   */
  public double getDegToApriltag(int id) {
    Measurement distX = getApriltagDistX(id);
    Measurement distY = getApriltagDistY(id);

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          /*
           * Takes Photonvision Z angle theta value (3D processing mode on camera) and
           * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
           * left the # of degs. that arcTan or inverse tan returns from the X & Y
           * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
           * coordinates. - TK
           */

          // Need to use the getX method that we wrote for Y in atan because it returns
          // the Photon Y. - TK
          if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
            return Math.toDegrees(Math.atan2(distX.distance, distY.distance));
          } else {
            return 999;
          }
        }
      }
      return 999;
    } else {
      return 9999;
    }
  }

  /**
   * Represents data related to an AprilTag.
   */
  public aprilTagData getAprilTagData(int id) {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          Measurement measurement = getApriltagDistY(id);

          if (measurement != null) {
            double deg = getDegToApriltag(id);

            return new aprilTagData(true, measurement.distance, measurement.ambiguity, deg, id);
          } else {
            return new aprilTagData(false, 0, 0, 0, -1);
          }
        }
      }
    }
    return new aprilTagData(false, 0, 0, 0, -1);
  }

  /**
   * Retrieves the layout of the AprilTag.
   *
   * @return the layout of the AprilTag
   */
  public aprilTagLayout getAprilTagLayout() {
    return aprilTagLayout;
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
   * @return The angle of the shape relative to the robot, or 999 if no shape is detected.
   */
  public double getShapeAngle() {
    PhotonTrackedTarget target = shape.getLatestResult().getBestTarget();

    // Robot relative angle
    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets() && target != null) {
      return target.getYaw();
    }
    return 999;
  }

  /**
   * Returns the area of the shape detected by the camera.
   * 
   * @return The area of the shape, or 0.0 if no shape is detected or the camera is not connected.
   */
  public double getShapeArea() {
    PhotonTrackedTarget target = shape.getLatestResult().getBestTarget();

    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets() && target != null) {
      return target.getArea();
    }
    return 0.0;
  }

  /**
   * Returns the timestamp of the camera image.
   *
   * @return The timestamp of the camera image.
   */
  public double getTimestamp() {
    return inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
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
   * Returns an optional EstimatedRobotPose object representing the estimated global pose of the robot.
   * The estimated global pose is obtained by setting the reference pose of the AprilTagPoseEstimator
   * to the current pose of the SwerveDrive instance and updating it with the latest result from the AprilTag.
   * 
   * @return an optional EstimatedRobotPose object representing the estimated global pose of the robot,
   *         or an empty optional if the pose cannot be estimated.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    aprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose()); // TODO: If this is a generic class it cannot depend on our drive train type.
    return aprilTagPoseEstimator.update(april.getLatestResult());
  }
}