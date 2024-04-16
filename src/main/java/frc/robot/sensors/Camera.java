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

  private double minAmbiguity;

  // TODO: Determine if we need the pathfind to apriltag code
  /*
   * // Global variables for updating Pathplanner poses - TK
   * private SwerveDrive swerveDrive;
   * private Pose2d currentSwervePose2d;
   * private double currentX;
   * private double currentY;
   * private double newX;
   * private double newY;
   *
   * 
   * // percentage of forward distance you want to drive before stopping (to
   * prevent
   * // crashing). - TK
   * private double percentTravelDist = 0.8; // Must be < 1
   */

  // Global fiducial ids for important landmark apriltags - TK
  public aprilTagLayout aprilTagLayout;
  private int speakerAprilTagC;
  private int ampAprilTag;
  private int sourceAprilTag;

  public class aprilTagLayout {
    public final int speakerAprilTag;
    public final int ampAprilTag;
    public final int sourceAprilTag;

    public aprilTagLayout(int speaker, int amp, int source) {
      this.speakerAprilTag = speaker;
      this.ampAprilTag = amp;
      this.sourceAprilTag = source;
    }
  }

  public class aprilTagData {
    public final boolean isDetected;
    public final double distance;
    public final double ambiguity;
    public final double angle;
    public final int id;

    public aprilTagData(boolean isDetected, double dist, double ambiguity, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.ambiguity = ambiguity;
      this.angle = angle;
      this.id = id;
    }
  }

  public class ShapeData {
    public final boolean isDetected;
    public final double distance;
    public final double angle;

    public ShapeData(boolean isDetected, double dist, double angle) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
    }
  }

  public class DistMeasurement {
    public final double distance;
    public final double ambiguity;

    public DistMeasurement(double distance, double ambiguity) {
      this.distance = distance;
      this.ambiguity = ambiguity;
    }
  }

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

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(5, 1, 0.1);
    }
    return instance;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera aprilGetInstance() {
    checkVersion();
    if (versionMatches && april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera notesGetInstance() {
    checkVersion();
    if (versionMatches && shape == null) {
      shape = new PhotonCamera(inst, "shape");
    }
    return shape;
  }

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

  private boolean testConnection() {
    // If either camera is working || connected == true
    if (april.isConnected() || shape.isConnected()) {
      connected = true;
    } else {
      connected = false;
    }

    return connected;
  }

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

  private void setNetworktableStatus() {
    try {
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

  public boolean getStatus() {
    setNetworktableStatus();

    // Just returns the boolean that shows connction status - TK
    if (versionMatches && connected) {
      return true;
    } else {
      return false;
    }
  }

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

  public boolean getAprilTagDetected() {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().hasTargets();
    }
    return false;
  }

  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getFiducialId();
    } else {
      return -1;
    }
  }

  public double getApriltagYaw() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getYaw();
    } else {
      return 999;
    }
  }

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

  public double getApriltagPitch() {
    // If this function returns a 999, that means there is not any detected targets
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      return target.getPitch();
    } else {
      return 999;
    }
  }

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

  public DistMeasurement getApriltagDistX() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new DistMeasurement(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistMeasurement getApriltagDistX(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new DistMeasurement(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  public DistMeasurement getApriltagDistY() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    PhotonTrackedTarget target = april.getLatestResult().getBestTarget();

    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets() && target != null) {
      return new DistMeasurement(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistMeasurement getApriltagDistY(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target != null && target.getFiducialId() == id) {
          return new DistMeasurement(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  /**************************************************************************************/

  public double getAprilTagDist() {
    DistMeasurement distX = getApriltagDistX();
    DistMeasurement distY = getApriltagDistY();

    if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
      return Math.sqrt((Math.pow(distX.distance, 2) + Math.pow(distY.distance, 2)));
    } else {
      return 0;
    }
  }

  public double getAprilTagDist(int id) {
    DistMeasurement distX = getApriltagDistX(id);
    DistMeasurement distY = getApriltagDistY(id);

    if (distX != null && distY != null && (distX.ambiguity + distY.ambiguity) / 2 <= minAmbiguity) {
      return Math.sqrt((Math.pow(distX.distance, 2) + Math.pow(distY.distance, 2)));
    } else {
      return 0;
    }
  }

  public double getDegToApriltag() {
    DistMeasurement distX = getApriltagDistX();
    DistMeasurement distY = getApriltagDistY();

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

  public double getDegToApriltag(int id) {
    DistMeasurement distX = getApriltagDistX(id);
    DistMeasurement distY = getApriltagDistY(id);

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

  public aprilTagData getAprilTagData(int id) {
    if (connected && versionMatches && april != null && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          DistMeasurement measurement = getApriltagDistY(id);

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

  public aprilTagLayout getAprilTagLayout() {
    return aprilTagLayout;
  }

  public boolean getShapeDetected() {
    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets()) {
      return shape.getLatestResult().hasTargets();
    }
    return false;
  }

  public double getShapeAngle() {
    PhotonTrackedTarget target = shape.getLatestResult().getBestTarget();

    // Robot relative angle
    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets() && target != null) {
      return target.getYaw();
    }
    return 999;
  }

  public double getShapeArea() {
    PhotonTrackedTarget target = shape.getLatestResult().getBestTarget();

    if (connected && versionMatches && shape != null && shape.getLatestResult().hasTargets() && target != null) {
      return target.getArea();
    }
    return 0.0;
  }

  public double getTimestamp() {
    return inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
  }

  public double getCurrentTime() {
    // Shouldn't need to typecast here, but JIC :) - TK
    return (double) (System.currentTimeMillis() - programStartTime);
  }

  public double getLatencySeconds() {
    return april.getLatestResult().getLatencyMillis() / 1000;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    aprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose());
    return aprilTagPoseEstimator.update(april.getLatestResult());
  }
}