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
    
    public DetectAprilTagL1(double timeout) {

        m_timeout = timeout;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.m_subsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // TODO System.out.print("INIT DETECT APRIL TAG");
        m_end_time = System.currentTimeMillis() / 1000.0 + m_timeout;

        m_distance_sum = 0;
        m_angle_sum = 0;
        m_count = 0;

        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            tag_id = 7; // TODO: Don't hardcode this.  Change it based on team assignment.
        }else{
            tag_id = 4;
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        try {
            
            target = m_camera.latestAprilTagDetection(tag_id);
            
            if (target != null) {
            double ambiguity = target.getPoseAmbiguity();
            if(ambiguity > 0.1) {
                RobotContainer.controller.setRumble().schedule();
                RobotContainer.controller2.setRumble().schedule();
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
            
            // TODO: Remove this print statement
            //System.out.println("AprilTag detected: " + tag_id + " at distance " + distance + " and angle " + angle + " and Yaw " + yaw);
            }else{
                RobotContainer.controller.setRumble().schedule();
                RobotContainer.controller2.setRumble().schedule();
            }
        }catch (Exception e) {
            System.err.println(e + "Line 92");
        }
    }
    public PhotonTrackedTarget getTarget(){
        return target;
    }

    public double getDistance() {
        return m_distance_sum / m_count;
    }   

    public double getYawAngle() {
        return m_angle_sum / m_count;
    }   

    public int count() {
        return m_count;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_count > 2 ) {
            // double avg_distance = m_distance_sum / m_count;
            // double avg_angle = m_angle_sum / m_count;
            // TODO: Remove this print statement
            //System.out.println("AprilTag average distance: " + avg_distance + " and angle " + avg_angle);
            return true;
        }

        if (System.currentTimeMillis() / 1000.0 > m_end_time) {
            // TODO: Remove this print statement
            //System.out.println("AprilTag timed out");
            return true;
        }

        return false;
    }
    
}
