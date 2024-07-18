package frc.robot.commands.L1Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.sensors.Camera;

/**
 * Detect the april tag
 * 
 * Positive yaw values mean turn the robot left.  Negitive yaw values mean turn the robot right.
 * 
 * Distance seems to be along the ground and over estimates by about 7 percent.
 */
public class DetectAprilTagL1 extends Command {

    private Camera m_camera = Camera.getInstance();

    private double m_distance_sum = 0;
    private double m_angle_sum = 0;
    private int m_count = 0;

    // start time
    private double m_end_time;
    private double m_timeout;
    private int tag_id = 0;
    PhotonTrackedTarget target;
    
    /**
     * This command is responsible for detecting an AprilTag in Level 1.
     * It takes a timeout value as a parameter.
     * 
     * @param timeout The time in seconds to run the command before it times out.
     */
    public DetectAprilTagL1(double timeout) {

        m_timeout = timeout;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.m_subsystem);
    }
    
    /**
     * Initializes the DetectAprilTagL1 command.
     * This method is called once when the command is first scheduled.
     * It sets up the necessary variables and assigns the tag ID based on the alliance color.
     */
    @Override
    public void initialize() {
        m_end_time = System.currentTimeMillis() / 1000.0 + m_timeout;

        m_distance_sum = 0;
        m_angle_sum = 0;
        m_count = 0;

        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            tag_id = 7; 
        }else{
            tag_id = 4;
        }
    }
    
    /**
     * Executes the command to detect an AprilTag and update the running sum of distance and angle.
     * If the pose of the detected tag is ambiguous, the measurement is skipped.
     * 
     * @throws Exception if an error occurs during the execution of the command.
     */
    @Override
    public void execute() {
        try {
            
            target = m_camera.latestAprilTagDetection(tag_id);
            
            if (target != null) {
            double ambiguity = target.getPoseAmbiguity();
            if(ambiguity > 0.1) {
                RobotContainer.driver_controller.setRumble().schedule();
                RobotContainer.operator_controller.setRumble().schedule();
                // Skip this measurement if the pose is ambiguous
                return;
            }
            Transform3d pose = target.getBestCameraToTarget();
            double x = pose.getX();
            double y = pose.getY();
            double distance = Math.sqrt(x * x + y * y);
            // double angle = Math.atan2(y, x);
            double yaw = target.getYaw();
            
            // Update the running sum
            m_distance_sum += distance;
            m_angle_sum += yaw;
            m_count++;
            
            }else{
                RobotContainer.driver_controller.setRumble().schedule();
                RobotContainer.operator_controller.setRumble().schedule();
            }
        }catch (Exception e) {
            System.err.println(e + "Line 92");
        }
    }

    /**
     * Returns the PhotonTrackedTarget object representing the detected target.
     *
     * @return The PhotonTrackedTarget object representing the detected target.
     */
    public PhotonTrackedTarget getTarget(){
        return target;
    }

    /**
     * Calculates and returns the average distance based on the sum of distances and the count of measurements.
     * 
     * @return The average distance.
     */
    public double getDistance() {
        return m_distance_sum / m_count;
    }   

    /**
     * Returns the average yaw angle calculated from the sum of angles and the count of measurements.
     *
     * @return the average yaw angle
     */
    public double getYawAngle() {
        return m_angle_sum / m_count;
    }   

    /**
     * Returns the count of values in the average.
     *
     * @return the count
     */
    public int count() {
        return m_count;
    }
    
    /**
     * This method is called when the command ends.
     * 
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
    }
    
    /**
     * Checks if the command has finished executing.
     * The command is considered finished if either the count exceeds 2 or the current time exceeds the end time.
     * @return true if the command is finished, false otherwise.
     */
    @Override
    public boolean isFinished() {
        if (m_count > 2 ) {
            return true;
        }

        if (System.currentTimeMillis() / 1000.0 > m_end_time) {
            return true;
        }

        return false;
    }
    
}
