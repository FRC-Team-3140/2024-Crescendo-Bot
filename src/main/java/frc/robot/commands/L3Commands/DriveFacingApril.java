package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class DriveFacingApril extends Command {
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
        // if(RobotContainer.controller.getXButton()){
        //     speed = maxSpeed /2;
        //     turnSpeed = maxChassisTurnSpeed/2;
        // }
        double yDist = SwerveDrive.getInstance().getPose().getY()- (216*.0254);
        double xDist = SwerveDrive.getInstance().getPose().getX();
        final var xSpeed = -RobotContainer.controller.getLeftY() * maxSpeed; // Calculate the x speed based on the joystick input
        final var ySpeed = -RobotContainer.controller.getLeftX() * maxSpeed; // Calculate the y speed based on the joystick input
        var angle = Math.asin(yDist/ SwerveDrive.getInstance().getDistanceFromSpeaker()); // Calculate the rotation speed based on the joystick input
        angle += 1/(yDist * yDist + 1)/ xDist * ySpeed * .02  - yDist/(xDist * xDist + 1)/xDist/xDist * xSpeed * .02;
        
        SmartDashboard.putNumber("ANgle", angle);
        SmartDashboard.putNumber("FDJKL", SwerveDrive.getInstance().getPose().getY()- (216*.0254)/ SwerveDrive.getInstance().getDistanceFromSpeaker());
        swerveDrive.drive(xSpeed, ySpeed, fieldRelative, angle); // Drive the swerve drive
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
