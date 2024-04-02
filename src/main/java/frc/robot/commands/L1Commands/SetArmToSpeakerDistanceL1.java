package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmToSpeakerDistanceL1 extends Command{

    private final double kAngleToleranceDegrees = 2.0;

    private Arm m_arm = Arm.getInstance();
    private DetectAprilTagL1 m_detect_april_tag;
    private double m_angle_setpoint = 0.0;

    public SetArmToSpeakerDistanceL1(DetectAprilTagL1 detectAprilTag) {
        m_detect_april_tag = detectAprilTag;
    }

    @Override
    public void initialize() {
        System.out.println("************ SetArmToSpeakerDistanceL1 ***********");
        System.out.println("   INIT SET ARM TO SPEAKER DISTANCE SetArmToSpeakerDistanceL1");
        // Assuming you have a method to set the arm to a distance
        if(m_detect_april_tag.count() >= 3) { // THIS HAS TO HAPPEN AFTER m_detect_april_tag IS COMPLETE
            double distance = m_detect_april_tag.getDistance();
            m_angle_setpoint = m_arm.estimateAngleForDistance(distance);
            System.out.println("   Distance: " + distance);
            System.out.println("   Angle:    " + m_angle_setpoint);
            m_arm.setArmToAngle(m_angle_setpoint);
        }
        else {
            // Do something else
            System.out.println("ERROR SetArmToSpeakerDistanceL1: Not enough detections");
        }
    }

    @Override
    public void execute() {
        // Nothing to do here
    }

    @Override
    public boolean isFinished() {
        // Assuming you have a way to check if the arm is at the correct distance
        double current_angle = m_arm.getAngle();
        return Math.abs(current_angle - m_angle_setpoint) < kAngleToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        // nothing to do here
    }
    
}
