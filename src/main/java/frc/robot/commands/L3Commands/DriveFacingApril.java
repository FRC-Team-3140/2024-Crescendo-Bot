package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class DriveFacingApril extends Command implements Constants {
    private final SwerveDrive swerveDrive; // The swerve drive subsystem
    private final double maxSpeed;
    public static boolean fieldRelative = true;

    public DriveFacingApril(SwerveDrive swerveDrive, double maxSpeed) {
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
        Translation2d speakerPose = DriverStation.getAlliance().get().equals(Alliance.Red) ? redSpeakerPose
                : blueSpeakerPose;
        double angle = Math.asin(SwerveDrive.getInstance().getExpectedPose().getY()
                / SwerveDrive.getInstance().getExpectedPose().getTranslation().getDistance(speakerPose));
        final var xSpeed = -RobotContainer.controller.getLeftY() * maxSpeed; // Calculate the x speed based on the
                                                                             // joystick input
        final var ySpeed = -RobotContainer.controller.getLeftX() * maxSpeed; // Calculate the y speed based on the
                                                                             // joystick input
        swerveDrive.driveFacingAngle(xSpeed, ySpeed, fieldRelative, angle); // Drive the swerve drive
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
