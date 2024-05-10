// package frc.robot.commands.L1Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Arm;

// /**
//  * This command sets the arm of the robot arm to a specific angle based on the distance to a speaker.
//  * It uses AprilTag detection to determine the distance and calculates the angle setpoint accordingly.
//  * The command initializes the arm to the desired angle and checks if it has reached the correct distance.
//  * 
//  * Note: This command assumes the existence of methods in the Arm and DetectAprilTagL1 classes.
//  */
// public class SetArmToSpeakerDistanceL1 extends Command{

//     private final double kAngleToleranceDegrees = 2.0;

//     private Arm m_arm = Arm.getInstance();
//     private DetectAprilTagL1 m_detect_april_tag;
//     private double m_angle_setpoint = 0.0;

//     /**
//      * This class represents a command to set the arm to the distance of the speaker for Level 1.
//      * It takes a DetectAprilTagL1 object as a parameter to detect the AprilTag.
//      */
//     public SetArmToSpeakerDistanceL1(DetectAprilTagL1 detectAprilTag) {
//         m_detect_april_tag = detectAprilTag;
//     }

//     /**
//      * Initializes the SetArmToSpeakerDistanceL1 command.
//      * This method sets the arm to a specific distance from a speaker.
//      * It checks if the number of April tag detections is greater than or equal to 3,
//      * and if so, it retrieves the distance from the April tag detection and estimates the angle for that distance.
//      * Finally, it sets the arm to the calculated angle.
//      * If there are not enough detections, an error message is printed.
//      */
//     @Override
//     public void initialize() {
//         // TODO: print statments to cleanup
//         System.out.println("************ SetArmToSpeakerDistanceL1 ***********");
//         System.out.println("   INIT SET ARM TO SPEAKER DISTANCE SetArmToSpeakerDistanceL1");
//         // Assuming you have a method to set the arm to a distance
//         if(m_detect_april_tag.count() >= 3) { // THIS HAS TO HAPPEN AFTER m_detect_april_tag IS COMPLETE
//             double distance = m_detect_april_tag.getDistance();
//             m_angle_setpoint = m_arm.estimateAngleForDistance(distance);
//             System.out.println("   Distance: " + distance);
//             System.out.println("   Angle:    " + m_angle_setpoint);
//             m_arm.setArmToAngle(m_angle_setpoint);
//         }
//         else {
//             // Do something else
//             System.out.println("ERROR SetArmToSpeakerDistanceL1: Not enough detections");
//         }
//     }

//     /**
//         * Executes the command.
//         * This method is called when the command is scheduled to run.
//         * It does not perform any actions in this case.
//         */
//     @Override
//     public void execute() {
//         // Nothing to do here
//     }

//     /**
//      * Checks if the arm is at the correct distance.
//      * 
//      * @return true if the arm is at the correct distance, false otherwise.
//      */
//     @Override
//     public boolean isFinished() {
//         // Assuming you have a way to check if the arm is at the correct distance
//         double current_angle = m_arm.getAngle();
//         return Math.abs(current_angle - m_angle_setpoint) < kAngleToleranceDegrees;
//     }

//     /**
//         * This method is called when the command ends.
//         * It is responsible for cleaning up any resources or state that the command may have acquired during its execution.
//         * 
//         * @param interrupted true if the command was interrupted, false otherwise
//         */
//     @Override
//     public void end(boolean interrupted) {
//         // nothing to do here
//     }
    
// }
