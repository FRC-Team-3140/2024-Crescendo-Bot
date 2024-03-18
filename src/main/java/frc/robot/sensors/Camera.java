// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.commands.pathfindToApriltag;
// import frc.robot.commands.turnToFaceApriltag;
import frc.robot.subsystems.SwerveDrive;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  // Gets initial instantiation of Cameras - TK
  private PhotonCamera april = aprilGetInstance();
  private PhotonCamera notes = notesGetInstance();

  // TODO: Find the actual postition of the cameras on the bot. - TK
  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center. - TK
  private Transform3d robotToApril = new Transform3d(new Translation3d(-Units.inchesToMeters(29) / 2, 0.0, 0.5),
      new Rotation3d(0, 0, Math.PI));

  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center. - TK
  // TODO: get measurements for note cam - TK : private Transform3d robotToNote =
  // new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonPoseEstimator aprilTagPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToApril);

  private boolean connected = false;
  private int connectionAttempts = 2;

  // The heartbeat is a value in the Photonvision Networktable that continually
  // changes.
  private double heartbeat = 0;

  // previousResult must start at a value that's not normally returned - TK
  private double previousResult = -1;

  // This thread allows this connection check to run in the background and not
  // block other Subsystems.
  private Thread attemptReconnection = new Thread(this::attemptToReconnect);

  // Must not start at 0 so check doesn't run early
  private int count = 1;

  // Time to delay periodic method Networktable connection check. IN
  // Mili-Seconds!!
  private double delayTime = 5000.0;

  // Time to delay connection attempts is in SECONDS! - TK
  private double attemptDelay;

  // Global variables for updating Pathplanner poses - TK
  private SwerveDrive swerveDrive;
  private Pose2d currentSwervePose2d;
  private double currentX;
  private double currentY;
  private double newX;
  private double newY;

  // percentage of forward distance you want to drive before stopping (to prevent
  // crashing). - TK
  private double percentTravelDist = 0.8; // Must be < 1

  // Global fiducial ids for important landmark apriltags - TK
  private aprilTagLayout aprilTagLayout;
  private int speakerAprilTag;
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

  public class aprilTagLocation {
    public final boolean isDetected;
    public final double distance;
    public final double angle;
    public final int id;

    public aprilTagLocation(boolean isDetected, double dist, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
      this.id = id;
    }
  }

  public class NoteLocation {
    public final boolean isDetected;
    public final double distance;
    public final double angle;

    public NoteLocation(boolean isDetected, double dist, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
    }
  }

  public class DistAmb {
    public final double distance;
    public final double ambiguity;

    public DistAmb(double distance, double ambiguity) {
      this.distance = distance;
      this.ambiguity = ambiguity;
    }
  }

  private Camera(SwerveDrive swerve, int PhotonvisionConnectionAttempts, double delayBetweenAttempts) {
    attemptDelay = delayBetweenAttempts;
    // TODO: Look at the following line
    aprilTagPoseEstimator.setReferencePose(new Pose2d(0, 0, new Rotation2d()));
    while (connected == false && connectionAttempts <= PhotonvisionConnectionAttempts) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
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

    if (connected) {
      aprilGetInstance();
      notesGetInstance();
    }

    // TODO: Change back to SwerveDrive.getInstance() as long as it doesn't cause
    // problems - TK
    swerveDrive = swerve;

    // Will also create a field layout object and set global variables for landmark
    // apriltags
    // as mentioned earlier. This is not to be confused with the Photonvision
    // apriltag layout
    // class! - TK
    configureTeam();
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(RobotContainer.swerve, 5, 1);
    }
    return instance;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera aprilGetInstance() {
    if (april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera notesGetInstance() {
    if (notes == null) {
      notes = new PhotonCamera(inst, "notes");
    }
    return notes;
  }

  private void configureTeam() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      speakerAprilTag = 4; // Apriltag on left
      ampAprilTag = 5;
      sourceAprilTag = 10; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTag, ampAprilTag, sourceAprilTag);
    } else {
      speakerAprilTag = 8; // Apriltag on left
      ampAprilTag = 6;
      sourceAprilTag = 2; // Apriltag on left

      aprilTagLayout = new aprilTagLayout(speakerAprilTag, ampAprilTag, sourceAprilTag);
    }
  }

  private boolean testConnection() {
    // Gets new result from april camera and test if it's equal to the previous
    // result
    heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);

    if (heartbeat == previousResult) {
      connected = false;
      heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
    } else {
      connected = true;
    }

    return connected;
  }

  private void attemptToReconnect() {
    System.out.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");

    while (connected == false) {
      if (testConnection() == true) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in " + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
      }

      aprilGetInstance();
      notesGetInstance();
    }
  }

  public boolean getStatus() {
    // Just returns the boolean that shows connction status - TK
    if (versionMatches && connected) {
      return true;
    } else {
      return false;
    }
  }

  private void setNetworktableStatus() {
    // TODO: ensure this publishes properly. Especially PhotonVersion.version! - TK
    inst.getTable("Vision").getSubTable("Status").getEntry("Version Matches: ").setBoolean(versionMatches);
    inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Version: ")
        .setString(inst.getTable("photonvision").getEntry("version").getString("2024.2.8"));
    inst.getTable("Vision").getSubTable("Status").getSubTable("Version Info").getEntry("Photon Lib Version: ")
        .setString(PhotonVersion.versionString);
    inst.getTable("Vision").getSubTable("Status").getEntry("Connection: ").setBoolean(connected);
    return connected;
  }

  @Override
  public void periodic() {
    previousResult = heartbeat;
    heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);

    // Was using Timer.delay() function here, but this caused issues with the other
    // subsystems...
    // Dividing by ideal 20 Ms roboRio loop time. - TK
    if ((count % (delayTime / 20)) == 0 && !attemptReconnection.isAlive() && testConnection() == false) {
      try {
        attemptReconnection.start();
      } catch (IllegalThreadStateException e) {
        System.out.println("Exception occured in Camera: \n" + e + "\nThread state: " + attemptReconnection.getState());
      }
    }

    count++;

    // TODO: Figure out why getAprilTagX(id) accasionally returns 0. - TK
    // // aprilTagLocation tag = getAprilTagLocation(speakerAprilTag);
    // inst.getTable("Vision").getSubTable("Camera").getEntry("ID: ").setInteger(tag.id);
    // inst.getTable("Vision").getSubTable("Camera").getEntry("Detected: ").setBoolean(tag.isDetected);
    // inst.getTable("Vision").getSubTable("Camera").getEntry("Dist: ").setDouble(tag.distance);
    // inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees: ").setDouble(tag.angle);
  }

  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  public double getApriltagYaw() {
    // If this function returns a 999, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getYaw();
    } else {
      return 999;
    }
  }

  public double getApriltagYaw(int id) {
    // If this function returns a 999, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
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

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getPitch();
    } else {
      return 999;
    }
  }

  public DistAmb getApriltagDistX() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      PhotonTrackedTarget target = april.getLatestResult().getBestTarget();
      return new DistAmb(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistX(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          return new DistAmb(target.getBestCameraToTarget().getY(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistY() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      PhotonTrackedTarget target = april.getLatestResult().getBestTarget();
      return new DistAmb(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
    } else {
      return null;
    }
  }

  public DistAmb getApriltagDistY(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          return new DistAmb(target.getBestCameraToTarget().getX(), target.getPoseAmbiguity());
        }
      }
      return null;
    } else {
      return null;
    }
  }

  /**************************************************************************************/

  public double getAprilTagDist() {
    return getAprilTagDist(1);
  }

  public double getAprilTagDist(double forwardScalePercent) {
    return Math.sqrt((Math.pow(getApriltagDistX(), 2) + Math.pow((forwardScalePercent * getApriltagDistY()), 2)));
  }

  public double getAprilTagDist(int id) {
    return getAprilTagDist(id, 1);
  }

  public double getAprilTagDist(int id, double forwardScalePercent) {
    return Math.sqrt((Math.pow(getApriltagDistX(id), 2) + Math.pow((forwardScalePercent * getApriltagDistY(id)), 2)));
  }

  public double getDegToApriltag() {
    return getAprilTagDist(1);
  }

  public double getDegToApriltag(double forwardScalePercent) {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && april.getLatestResult().hasTargets()) {
      // double targetYaw = getApriltagYaw();
      double requiredTurnDegrees;

      /*
       * Takes Photonvision Z angle theta value (3D processing mode on camera) and
       * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
       * left the # of degs. that arcTan or inverse tan returns from the X & Y
       * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
       * coordinates. - TK
       */

      // if (Math.signum(targetYaw) == -1) {
      // requiredTurnDegrees = -Math.toDegrees(Math.atan2(getApriltagDistY(),
      // getApriltagDistX()));
      // } else {
      // requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistY(),
      // getApriltagDistX()));
      // }

      // Need to use the getX method that we wrote for Y in atan because it returns
      // the Photon Y. - TK
      requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX().distance, (forwardScalePercent * getApriltagDistY().distance)));

      return requiredTurnDegrees;
    } else {
      return 0;
    }
  }

  public double getDegToApriltag(int id) {
    return getAprilTagDist(id, 1);
  }

  public double getDegToApriltag(int id, double forwardScalePercent) {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          double requiredTurnDegrees;

          /*
           * Takes Photonvision Z angle theta value (3D processing mode on camera) and
           * gets sign, if sign is negative (aprilTag is on left of frame), it will turn
           * left the # of degs. that arcTan or inverse tan returns from the X & Y
           * coorinates. Else it turns right by the arcTan or inverse tan of the X & Y
           * coordinates. - TK
           */

          // Need to use the getX method that we wrote for Y in atan because it returns
          // the Photon Y. - TK
          requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX(id).distance, getApriltagDistY(id).distance));

          return requiredTurnDegrees;
        }
      }
      return 0;
    } else {
      return 0;
    }
  }

  /* TODO: Broken calls 
  public aprilTagLocation getAprilTagLocation(int id) {
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          double dist = getApriltagDistY(id).distance;
          double deg = getDegToApriltag(id);

          return new aprilTagLocation(true, dist, deg, id);
        }
      }
    }
    return new aprilTagLocation(false, 0, 0, -1);
  }*/

  public aprilTagLayout getAprilTagLayout() {
    // TODO: Check to see if this variable matches the one I just added for the
    // photonvision pose estimator - TK
    return aprilTagLayout;
  }

  public double getNoteDistance() {
    // If this function returns a 0, that means there is not any detected targets

    // Need to wait until cameras are on Final Robot because calculation requires
    // specific
    // measurements to the camera.

    notes.getLatestResult().getBestTarget();
    PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);

    return 0.0;
  }

  public double getTimestamp() {
    return (1.0 * heartbeat);
  }

  public double getLatency() {
    return april.getLatestResult().getLatencyMillis();
  }

  double lastResult = 0;

  public boolean isConnected() {
    if (!connected) {
      return false;
    }
    boolean kjasdfl = april.getLatestResult().hasTargets() && connected
        && april.getLatestResult().getTimestampSeconds() != lastResult;
    lastResult = april.getLatestResult().getTimestampSeconds();
    return kjasdfl;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    aprilTagPoseEstimator.setReferencePose(SwerveDrive.getInstance().getPose());
    return aprilTagPoseEstimator.update(april.getLatestResult());
    // aprilTagPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // if (connected && april.getLatestResult().hasTargets() &&
    // !april.getLatestResult().equals(lastResult)){
    // } else {
    // return null;
    // }
  }

  // public SequentialCommandGroup pathfindToAprilTag() {
  // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // new turnToFaceApriltag(speakerAprilTag, RobotContainer.swerve,
  // Camera.getInstance()),
  // new InstantCommand(() -> {
  // currentSwervePose2d = RobotContainer.swerve.getPose();
  // currentX = currentSwervePose2d.getX();
  // currentY = currentSwervePose2d.getY();
  // newX = getApriltagDistX(speakerAprilTag);
  // newY = percentTravelDist * getApriltagDistY(speakerAprilTag);
  // }),
  // new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new
  // Rotation2d(0)),
  // Camera.getInstance(),
  // RobotContainer.swerve),
  // new turnToFaceApriltag(speakerAprilTag, RobotContainer.swerve,
  // Camera.getInstance()));

  // // Fallback code. NOT tested!!!!! - TK
  // // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // // // new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(4.5, 6.5, new
  // //
  // Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees()))))),
  // // new InstantCommand(() -> System.out.println("X: " +
  // // swerveDrive.getPose().getX() + " Y: " + swerveDrive.getPose().getY())),
  // // // new Pathfinding(new Pose2d((swerveDrive.getPose().getX()/* + 0.25*/),
  // // (swerveDrive.getPose().getY()/* + 0.25*/), new
  // // Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees()
  // +
  // // 45))), Camera.getInstance(), swerveDrive),
  // // new InstantCommand(() -> System.out.println("DistX to Target:
  // // "+getApriltagDistX(2))),
  // // // new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0,
  // // Math.signum(getApriltagDistX(2)), getDegToApriltag(2), false)),
  // // new InstantCommand(() ->
  // // RobotContainer.swerve.drive(0,1.0*Math.signum(getApriltagDistX(2)), 0,
  // // false)),
  // // new InstantCommand(() -> System.out.println("DONE"))
  // // );

  // return goToAprilTag;
  // }

  // public SequentialCommandGroup pathfindToAprilTag(int id) {
  // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
  // new turnToFaceApriltag(id, swerveDrive, Camera.getInstance()),
  // new InstantCommand(() -> {
  // currentSwervePose2d = swerveDrive.getPose();
  // currentX = currentSwervePose2d.getX();
  // currentY = currentSwervePose2d.getY();
  // newX = getApriltagDistX(id);
  // newY = percentTravelDist * getApriltagDistY(id);
  // }),
  // new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new
  // Rotation2d(0)),
  // Camera.getInstance(),
  // swerveDrive),
  // new turnToFaceApriltag(swerveDrive, Camera.getInstance()));

  // return goToAprilTag;
  // }
}