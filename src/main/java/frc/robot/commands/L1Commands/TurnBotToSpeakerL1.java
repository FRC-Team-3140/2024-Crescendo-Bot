package frc.robot.commands.L1Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class represents a command to turn the robot to a specific angle.
 * It extends the Command class from WPILib and uses a PID controller to manage
 * the turning motion.
 */
public class TurnBotToSpeakerL1 extends Command {
    // The swerve drive subsystem of the robot
    SwerveDrive m_swerveDrive;

    // Constants for the PID controller and the angle tolerance
    private final double kAngleToleranceDegrees = 2.0;
    private final double kP = 0.05;
    private final double kI = 0.0; // Leave as zero.
    private final double kD = 0.0;

    // The PID controller for the turning motion
    private PIDController m_turnController;

    private DetectAprilTagL1 m_detect_april_tag;

    // Variables to store the relative angle, current angle, and setpoint angle
    private double m_start_angle = 0.0;
    private double m_setpoint_angle = 0.0;

    private boolean m_first_execute = true;

    // My network table
    NetworkTable m_table = NetworkTableInstance.getDefault().getTable("TurnBotToAngleL1");

    /**
     * Creates a new TurnBotToSpeakerL1 command.
     * This command turns the robot to a specific angle using a PID controller.
     * 
     * @param detectAprilTag the command that detects the AprilTag
     */
    public TurnBotToSpeakerL1(DetectAprilTagL1 detectAprilTag) {
        m_swerveDrive = SwerveDrive.getInstance();

        // Store the DetectAprilTagL1 command
        m_detect_april_tag = detectAprilTag;

        // Initialize the PID controller with the PID constants
        m_turnController = new PIDController(kP, kI, kD);

        // Since this controller sets an angle it should be continuous
        m_turnController.enableContinuousInput(-180, 180);
        m_turnController.setTolerance(kAngleToleranceDegrees);

        try {
            m_table.getEntry("setpoint_angle").setDouble(m_setpoint_angle);
            m_table.getEntry("current_angle").setDouble(m_start_angle);
            m_table.getEntry("turn_speed").setDouble(0.0);
        } catch (Exception e) {
            System.out.println("Error with network table");
        }

        // This command requires the swerve drive subsystem
        addRequirements(m_swerveDrive);
    }

    /**
     * Initializes the command.
     * This method is called once when the command is first scheduled.
     * It sets the start angle to 0 and sets the first execute flag to true.
     */
    @Override
    public void initialize() {
        // System.out.println("INIT TURN TO SPEAKER");
        // Not needed for this command
        m_start_angle = 0;
        m_first_execute = true;
    }

    /**
     * Executes the command to turn the robot towards the speaker.
     * This method calculates the target angle based on the detected yaw angle of
     * the speaker,
     * and uses a PID controller to adjust the robot's turning speed accordingly.
     * The robot's current angle, setpoint angle, and turn speed are printed for
     * debugging purposes.
     * The robot is then driven with the calculated turn speed.
     * The current angle and turn speed are also sent to a network table for
     * monitoring.
     */
    @Override
    public void execute() {
        // compute this the first time through the execute loop
        if (m_first_execute) { // THIS HAS TO HAPPEN AFTER m_detect_april_tag IS COMPLETE
            m_first_execute = false;
            m_start_angle = m_swerveDrive.getGyroAngle();
            double target_angle = m_detect_april_tag.getYawAngle();
            m_setpoint_angle = target_angle + m_start_angle;
            // TODO System.out.println("FIRST EXECUTE");
            // System.out.println("TARGET_ANGLE: " + target_angle);
            m_turnController.setSetpoint(m_setpoint_angle);
        }

        // Update the current angle
        double current_angle = m_swerveDrive.getGyroAngle();

        // Calculate the speed to turn at based on the current angle
        double turn_speed = m_turnController.calculate(current_angle, m_setpoint_angle);

        // Print the useful information for debugging
        // TODO System.out.println("============== TURNING ==============");
        // System.out.println(" Current Angle: " + current_angle);
        // System.out.println(" Setpoint Angle: " + m_setpoint_angle);
        // System.out.println(" Turn Speed: " + turn_speed);
        // System.out.println(" Start Angle: " + m_start_angle);
        // System.out.println(" At Setpoint: " + m_turnController.atSetpoint());
        // System.out.println("===================================");

        // Drive the robot with the calculated turn speed
        m_swerveDrive.drive(0, 0, -turn_speed, true);

        try {
            m_table.getEntry("current_angle").setDouble(m_start_angle);
            m_table.getEntry("turn_speed").setDouble(turn_speed);
        } catch (Exception e) {
            System.out.println("Error with network table");
        }
    }

    /**
     * Checks if the command is finished.
     * The command is considered finished when the PID controller reaches the
     * setpoint.
     *
     * @return true if the command is finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        // The command is finished when the PID controller is at the setpoint
        return m_turnController.atSetpoint();
    }

    /**
     * Called when the command ends.
     * Stops the robot by setting the swerve drive velocities to zero.
     * 
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_swerveDrive.drive(0, 0, 0, true);
    }
}