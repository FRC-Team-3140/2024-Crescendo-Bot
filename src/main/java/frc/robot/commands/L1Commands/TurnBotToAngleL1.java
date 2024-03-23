package frc.robot.commands.L1Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class represents a command to turn the robot to a specific angle.
 * It extends the Command class from WPILib and uses a PID controller to manage the turning motion.
 */
public class TurnBotToAngleL1 extends Command {
    // The swerve drive subsystem of the robot
    SwerveDrive m_swerveDrive;

    // Constants for the PID controller and the angle tolerance
    private final double kAngleToleranceDegrees = 2.0;
    private final double kP = 0.005;
    private final double kI = 0.0; // Leave as zero.
    private final double kD = 0.0;

    // The PID controller for the turning motion
    private PIDController m_turnController;

    // Variables to store the relative angle, current angle, and setpoint angle
    private double m_relative_angle;
    private double m_current_angle;
    private double m_setpoint_angle;

    /**
     * Constructor for the TurnBotToAngleL1 command.
     * @param relative_angle The angle to turn to, relative to the current angle.
     */
    public TurnBotToAngleL1(double relative_angle) {
        m_swerveDrive = SwerveDrive.getInstance();

        m_relative_angle = relative_angle;
        m_current_angle = m_swerveDrive.getGyroAngle();

        // Calculate the setpoint angle based on the current angle and the relative angle
        m_setpoint_angle = m_current_angle + m_relative_angle;

        // Initialize the PID controller with the PID constants
        m_turnController = new PIDController(kP, kI, kD);

        // Since this controller sets an angle it should be continuous
        m_turnController.enableContinuousInput(-180, 180);
        m_turnController.setTolerance(kAngleToleranceDegrees);

        // This command requires the swerve drive subsystem
        addRequirements(m_swerveDrive);
    }   

    @Override
    public void initialize() {
        // Not needed for this command
    }

    @Override
    public void execute() {
        // Update the current angle
        m_current_angle = m_swerveDrive.getGyroAngle();

        // Calculate the speed to turn at based on the current and setpoint angles
        double turn_speed = m_turnController.calculate(m_current_angle, m_setpoint_angle);

        // Drive the robot with the calculated turn speed
        m_swerveDrive.drive(0, 0, turn_speed, true);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the PID controller is at the setpoint
        return m_turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_swerveDrive.drive(0, 0, 0, true);
    }
}