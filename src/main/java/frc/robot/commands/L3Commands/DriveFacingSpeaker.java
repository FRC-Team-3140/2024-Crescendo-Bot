package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
//Works Well

/**
 * This class represents a basic swerve control command.
 * It is intended to be the default command for the drive.
 */
public class DriveFacingSpeaker extends Command {
    private final SwerveDrive swerveDrive; // The swerve drive subsystem
    private final double maxSpeed; // The maximum speed for the swerve drive
    public static boolean fieldRelative = true;

    /**
     * Creates a new BasicSwerveControlL2 command.
     *
     * @param swerveDrive         The swerve drive subsystem
     * @param maxSpeed            The maximum speed for the swerve drive
     * @param maxChassisTurnSpeed The maximum turn speed for the chassis
     */
    public DriveFacingSpeaker(SwerveDrive swerveDrive, double maxSpeed) {
        this.swerveDrive = swerveDrive;
        this.maxSpeed = maxSpeed;
        addRequirements(swerveDrive); // This command requires the swerve drive subsystem

    }

    /**
     * The command execution logic.
     * Gets the joystick inputs and drives the swerve drive accordingly.
     */
    @Override
    public void execute() {
        final var xSpeed = -RobotContainer.controller.getLeftY() * maxSpeed; // Calculate the x speed based on the
                                                                             // joystick input
        final var ySpeed = -RobotContainer.controller.getLeftX() * maxSpeed; // Calculate the y speed based on the
                                                                             // joystick input
        swerveDrive.driveRobotFacingSpeaker(xSpeed, ySpeed, fieldRelative); // Drive the swerve drive
    }

    /**
     * Determines whether the command is finished.
     * If this command is the default command for the drive, it should never finish.
     *
     * @return false because this command should never finish if it's the default
     *         command for the drive
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}