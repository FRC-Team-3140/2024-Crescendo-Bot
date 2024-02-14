// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

// TODO: Not Complete. At some point, replace with Tylers finished code.

/* TODO: These are suggestions from copilot on refactoring and simplifying this class. - DB
* 
* This class is indeed quite large and complex. Here are some suggestions to simplify it:
* 
* 1. **Separate concerns**: This class seems to be doing a lot of things - managing connections, handling camera instances, and dealing with AprilTag locations. Consider breaking it down into smaller classes, each with a single responsibility.
* 
* 2. **Avoid using magic numbers**: There are several numbers in the code (like 999, 0.8, 2, etc.) that could be replaced with named constants to make the code more readable and maintainable.
*
* 3. **Use a connection manager**: The connection handling logic could be moved to a separate class or method to simplify the main class.
*
* 4. **Use a factory pattern for camera instances**: Instead of having `aprilGetInstance` and `notesGetInstance` methods, consider using a factory method that takes the camera name as a parameter.
* 
* 5. **Avoid using threads directly**: The use of a Thread for reconnection attempts could be replaced with a higher-level concurrency construct, like a ScheduledExecutorService, to make the code safer and easier to manage.
* 
* 6. **Refactor repeated code**: There are several places where the same or similar code is repeated (like the code for getting AprilTag data). This could be refactored into helper methods to reduce duplication.
* 
* 7. **Use data classes for AprilTag data**: Instead of having separate methods for each piece of AprilTag data, consider creating a data class (or struct, depending on your language) to hold all the data for an AprilTag. This would simplify the interface of the class and make the code easier to understand.
*
* 8. **Use logging instead of System.out.println**: Replace all the `System.out.println` and `System.err.println` calls with proper logging. This will give you more control over the output and make it easier to manage.
*
* 9. **Handle exceptions properly**: There are places where exceptions are caught and printed, but not actually handled. Make sure to handle exceptions in a way that makes sense for your application.
* 
*10. **Use meaningful variable names**: Some variable names are not very descriptive (like `inst` and `april`). Using more descriptive names can make the code easier to understand.
*/

/**
 * This class is used to manage the Photonvision cameras and their connections.
 * It also provides methods to get the location of the speaker's apriltag,
 * detect notes for the intake camera, and find the robots pose for path
 * finding.
 */
public class Camera extends SubsystemBase {

    private static Camera instance = null;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // TODO: Refactor to apriltag_camera and note_camera - DB
    private PhotonCamera april = null;
    private PhotonCamera notes = null;

    private boolean connected = false;
    private int connectionAttempts = 1;

    // The heartbeat is a value in the Photonvision Networktable that continually
    // changes.
    private double heartbeat = 0;
    private double previousResult = -1;

    // This thread allows this connection check to run in the background and not
    // block
    // other Subsystems.
    private Thread attemptReconnection = new Thread(this::attemptToReconnect);

    // Must start not at 0 so check doesn't run early
    private int count = 1;

    // Time to delay periodic Networktable connection check. IN Mili-Seconds!!
    private double delayTime = 2000.0;

    // Time to delay connection attempts is in SECONDS! - TK
    private double attemptDelay;

    private SwerveDrive swerveDrive;
    private Pose2d currentSwervePose2d;
    private double currentX;
    private double currentY;
    private double newX;
    private double newY;

    private double percentTravelDist = 0.8; // Must be < 1

    // variables used to set the camera system up based on the robots team
    // assignment
    enum Team {
        UNKNOWN, RED, BLUE
    };

    private Team currentTeam = Team.UNKNOWN;
    private int speakerAprilTag = -1;
    private int ampAprilTag = -1;
    private int pickupAprilTag = -1;

    public class aprilTagLocation {
        // TODO: variable names can be improved. Lock at "NoteLocation" - DB
        public final boolean aprilTagdetected;
        public final double aprilTagdistance;
        public final double aprilTagAngle;
        public final int aprilTagID;

        public aprilTagLocation(boolean isDetected, double dist, double angle, int id) {
            aprilTagdetected = isDetected;
            aprilTagdistance = dist;
            aprilTagAngle = angle;
            aprilTagID = id;
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

    private Camera(SwerveDrive swerve, int PhotonvisionConnectionAttempts, double delayBetweenAttempts) {
        attemptDelay = delayBetweenAttempts;

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

        if (connected == true) {
            aprilGetInstance();
            notesGetInstance();
        }

        swerveDrive = swerve;
    }

    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera(RobotContainer.swerve, 5, 1);
        }
        return instance;
    }

    /**
     * Sets up the team assignment for the camera.
     * 
     * @param team The team to assign (RED or BLUE).
     */
    public void setupTeamAssignment(Team team) {
        currentTeam = team;
        // TODO: Check these values. They may be incorrect. - DB
        if (currentTeam == Team.RED) {
            speakerAprilTag = 4;
            ampAprilTag = 5;
            pickupAprilTag = 10;
        } else if (currentTeam == Team.BLUE) {
            speakerAprilTag = 7;
            ampAprilTag = 6;
            pickupAprilTag = 1;
        }
    }

    // TODO: This looks like a singleton pattern, but it's not. This is private
    // inside a class. Just create the apriltag camera in the constructor. Delete
    // this method - DB
    private PhotonCamera aprilGetInstance() {
        if (april == null) {
        }
        return april;
    }

    // TODO: This looks like a singleton pattern, but it's not. This is private
    // inside a class. Just create the note camera in the constructor. Delete this
    // method - DB
    private PhotonCamera notesGetInstance() {
        if (notes == null) {
            notes = new PhotonCamera(inst, "notes");
        }
        return notes;
    }

    private boolean testConnection() {
        // Gets new result from april camera and test if it's equal to the previous
        // result
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
        // System.out.println(heartbeat);
    }

    public boolean getStatus() {
        return connected;
    }

    @Override
    public void periodic() {
        previousResult = heartbeat;
        heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);

        // Was using Timer.delay() function here, but this caused issues with the other
        // subsystems...
        if ((count % delayTime) == 0 && !attemptReconnection.isAlive() && testConnection() == false) {
            try {
                attemptReconnection.start();
            } catch (IllegalThreadStateException e) {
                System.out.println(
                        "Exception occured in Camera: \n" + e + "\nThread state: " + attemptReconnection.getState());
            }
        }

        count++;

        aprilTagLocation tag = getAprilTagLocation(speakerAprilTag);
        inst.getTable("Vision").getSubTable("Camera").getEntry("ID: ").setInteger(tag.aprilTagID);
        inst.getTable("Vision").getSubTable("Camera").getEntry("Detected: ").setBoolean(tag.aprilTagdetected);
        inst.getTable("Vision").getSubTable("Camera").getEntry("Dist: ").setDouble(tag.aprilTagdistance);
        inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees: ").setDouble(tag.aprilTagAngle);
    }

    // TODO: These four methods are really what is needed to automate most of what
    // we need. - DB
    /**
     * This method returns the location of the speaker's apriltag. Make sure the
     * result is detected before using the data.
     * relative to the robot's current position.
     * 
     * TODO: Double check that this method is working as expected. - DB
     * 
     * @return The location of the speaker's apriltag relative to the robot.
     */
    public aprilTagLocation getSpeakerLocation() {
        return getAprilTagLocation(speakerAprilTag);
    }

    /**
     * This method returns the location of a note in the environment.
     *
     * The location is represented as a NoteLocation object, which contains
     * information about whether a note is detected,
     * the distance to the note, and the angle to the note from the current
     * position.
     *
     * TODO: Currently, this method returns a default NoteLocation object with all
     * values set to 0 and detection set to false.
     * This should be updated to return actual note location data when available.
     *
     * @return A NoteLocation object representing the location of a note.
     */
    public NoteLocation getNoteLocation() {
        return new NoteLocation(false, 0, 0, 0);
    }

    /**
     * Retrieves the location of the AprilTag for the amp. Make sure the result is
     * detected before using the data.
     * 
     * TODO: Double check that this method is working as expected. - DB
     * 
     * @return The location of the AprilTag relative to the robot.
     */
    public aprilTagLocation getAmpLocation() {
        return getAprilTagLocation(ampAprilTag);
    }

    /**
     * Retrieves the location of the AprilTag for the pickup. Make sure the result
     * is detected before using the data.
     *
     * TODO: Double check that this method is working as expected. - DB
     * 
     * @return The location of the AprilTag relative to the robot.
     */
    public aprilTagLocation getPickupLocation() {
        return getAprilTagLocation(pickupAprilTag);
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

    public double getApriltagDistX() {
        // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
        // phase.
        if (connected && april.getLatestResult().hasTargets()) {
            return april.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
        } else {
            return 0;
        }
    }

    public double getApriltagDistX(int id) {
        // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
        // phase.
        if (connected && april.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
                if (target.getFiducialId() == id) {
                    return target.getBestCameraToTarget().getY();
                }
            }
            return 0;
        } else {
            return 0;
        }
    }

    public double getApriltagDistY() {
        // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
        // phase.
        if (connected && april.getLatestResult().hasTargets()) {
            return percentTravelDist * april.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
        } else {
            return 0;
        }
    }

    public double getApriltagDistY(int id) {
        // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
        // phase.
        if (connected && april.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
                if (target.getFiducialId() == id) {
                    return percentTravelDist * target.getBestCameraToTarget().getX();
                }
            }
            return 0;
        } else {
            return 0;
        }
    }

    public double getAprilTagDist() {
        double dist;

        dist = Math.sqrt((Math.pow(getApriltagDistX(), 2) + Math.pow(getApriltagDistY(), 2)));

        return dist;
    }

    public double getDegToApriltag() {
        // Usable range of values with best consistancy: -50 - 50 With respect to
        // camera. - TK
        if (connected && april.getLatestResult().hasTargets()) {
            // double targetYaw = getApriltagYaw();
            double requiredTurnDegrees;

            /*
             * Takes Photonvision Z angle theta value (3D processing mode on camera) and
             * gets sign,
             * if sign is negative (apriltag is on left of frame), it will turn left the #
             * of degs.
             * that arcTan or inverse tan returns from the X & Y coorinates. Else it turns
             * right
             * by the arcTan or inverse tan of the X & Y coordinates. - TK
             */

            // if (Math.signum(targetYaw) == -1) {
            // requiredTurnDegrees = -Math.toDegrees(Math.atan2(getApriltagDistY(),
            // getApriltagDistX()));
            // } else {
            // requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistY(),
            // getApriltagDistX()));
            // }

            requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX(), getApriltagDistY()));

            System.out.println(requiredTurnDegrees);

            return requiredTurnDegrees;
        } else {
            return 0;
        }
    }

    public double getDegToApriltag(int id) {
        // Usable range of values with best consistancy: -50 - 50 With respect to
        // camera. - TK
        if (connected && april.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
                if (target.getFiducialId() == id) {
                    double requiredTurnDegrees;

                    /*
                     * Takes Photonvision Z angle theta value (3D processing mode on camera) and
                     * gets sign,
                     * if sign is negative (apriltag is on left of frame), it will turn left the #
                     * of degs.
                     * that arcTan or inverse tan returns from the X & Y coorinates. Else it turns
                     * right
                     * by the arcTan or inverse tan of the X & Y coordinates. - TK
                     */

                    requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX(id), getApriltagDistY(id)));

                    return requiredTurnDegrees;
                }
            }
            return 0;
        } else {
            return 0;
        }
    }

    // TODO: It is unclear what this method is for? Robot centric or estimate field
    // location? - DB
    public Pose2d getApriltagPose2d() {
        return new Pose2d(new Translation2d(getApriltagDistX(), getApriltagDistY()),
                new Rotation2d(getDegToApriltag()));
    }

    // TODO: Estimate field location from apriltags? I think this is what you need
    // for path planning and odometer updates. - DB

    /**
     * Enumeration representing different methods for pose estimation.
     * The available methods are:
     * - CENTERED_TAG: Estimates the pose based on the center of the detected
     * object.
     * - AVERAGE: Estimates the pose based on the average position of multiple
     * detected objects.
     * - CLOSEST_TAG: Estimates the pose based on the closest detected object.
     * - HIGHEST_CONFIDENCE: Estimates the pose based on the object with the highest
     * confidence level.
     */
    enum PoseEstimationMethod {
        CENTERED_TAG, CLOSEST_TAG, HIGHEST_CONFIDENCE, AVERAGE
    };

    /**
     * This method returns the robot pose based on the "best" detected AprilTag,
     * according to the specified pose estimation method.
     * 
     * TODO: This is my attempt at writing the method you need. Probably not correct
     * but maybe it will help. Getting the coordinate transforms between field
     * centric and robot centric coordinates will be very hard. Work on the easier
     * things first. - DB
     * 
     * Background Reading:
     * -
     * https://medium.com/@itberrios6/coordinate-transforms-for-sensor-fusion-40de05a6acf4
     * -
     * https://towardsdatascience.com/the-one-stop-guide-for-transformation-matrices-cea8f609bdb1
     * - https://www.youtube.com/watch?v=NWOL8yXL6xI
     * 
     * The pose estimation method can be one of the following:
     * - CENTERED_TAG: The "best" tag is the one with the smallest absolute yaw
     * (i.e., the one directly in front of the robot).
     * - CLOSEST_TAG: The "best" tag is the one with the smallest distance to the
     * robot.
     * - HIGHEST_CONFIDENCE: The "best" tag is the one with the highest detection
     * confidence (i.e., the largest area).
     * - AVERAGE: The "best" pose is the average of the poses of all detected tags.
     *
     * If no targets are detected, this method returns a default Pose2d object.
     *
     * @param method The pose estimation method to use.
     * @return A Pose2d object representing the pose of the "best" detected
     *         AprilTag, or a default Pose2d object if no targets are detected.
     */
    public Pose2d getBestAprilTagRobotPose(PoseEstimationMethod method) {
        PhotonTrackedTarget bestTarget = null;

        if (april.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
                if (bestTarget == null) {
                    bestTarget = target;
                } else {
                    switch (method) {
                        case CENTERED_TAG:
                            // TODO: Check this.
                            // Assuming the centered tag is the one with the smallest absolute yaw
                            if (Math.abs(target.getYaw()) < Math.abs(bestTarget.getYaw())) {
                                bestTarget = target;
                            }
                            break;
                        case CLOSEST_TAG:
                            // TODO: Check this.
                            // Assuming the closest tag is the one with the smallest distance
                            if (target.getBestCameraToTarget().getY() < bestTarget.getBestCameraToTarget().getY()) {
                                bestTarget = target;
                            }
                            break;
                        case HIGHEST_CONFIDENCE:
                            // TODO: Check this.
                            // Assuming the highest confidence tag is the one with the smallest pose
                            // ambiguity
                            if (target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity()) {
                                bestTarget = target;
                            }
                            break;
                        case AVERAGE:
                            // For average, you would need to average the poses of all targets
                            // TODO: This is a bit more complex and is left as an exercise
                            break;
                    }
                }
            }
        }

        if (bestTarget != null) {
            // TODO: Check this math. It could be completely wrong. I think the transform is
            // in the camera frame, so you may need to invert it to get the robot frame. -
            // DB
            // TODO: Do we need a table of apriltag field locations or is that provide by
            // Photonvision? - DB

            Transform3d transform = bestTarget.getBestCameraToTarget();
            // bestTarget.getAlternateCameraToTarget(); // TODO: If ambigutity is high this
            // may be a better choice.
            // transform = transform.inverse(); // TODO: The inverse may translate between
            // camera and april tag coordinates.
            return new Pose2d(new Translation2d(transform.getX(), transform.getY()),
                    transform.getRotation().toRotation2d());
        }

        return null;
    }

    /**
     * Retrieves the location of an AprilTag with the specified ID.
     * 
     * TODO: Keep this method. The other methods can call this one for all robot
     * centered actions. - DB
     * TODO: If this works I think you can move just the code needed for this method
     * here and delete the other apriltag methods. - DB
     * 
     * @param id The ID of the AprilTag to locate.
     * @return The location of the AprilTag as an instance of the aprilTagLocation
     *         class.
     */
    public aprilTagLocation getAprilTagLocation(int id) {
        if (april.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
                if (target.getFiducialId() == id) {
                    double x = target.getBestCameraToTarget().getX();
                    double y = target.getBestCameraToTarget().getY();
                    double dist = Math.sqrt(x * x + y * y);

                    // TODO: double check this. You might need to use the negitive of the x or y to
                    // get the correct angle. - DB
                    double deg = Math.toDegrees(Math.atan2(x, y));

                    return new aprilTagLocation(true, dist, deg, id);
                }
            }
        }
        return new aprilTagLocation(false, 0, 0, -1);
    }

    // TODO: move this code to the getNoteLocation method. Clean up the rest. - DB
    public double getNoteDistance() {
        // If this function returns a 0, that means there is not any detected targets

        // Need to wait until cameras are on Final Robot because calculation requires
        // specific
        // measurements to the camera.

        notes.getLatestResult().getBestTarget();
        PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);

        return 0.0;
    }

    // TODO: Deleted path finding for now so this compiles. Path finding commands should probably go in thier own classes. - DB
}