package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class DriveFacingApril extends Command{
     private final SwerveDrive swerveDrive; // The swerve drive subsystem
    private final double maxSpeed; // The maximum speed for the swerve drive
    private final double maxChassisTurnSpeed; // The maximum turn speed for the chassis

    
    public DriveFacingApril(SwerveDrive swerveDrive, double maxSpeed, double maxChassisTurnSpeed) {
        this.swerveDrive = swerveDrive;
        this.maxSpeed = maxSpeed;
        this.maxChassisTurnSpeed = maxChassisTurnSpeed;
        addRequirements(swerveDrive); // This command requires the swerve drive subsystem
        
    }
    
    @Override
    public void execute() {
        // final var xSpeed = -RobotContainer.controller.getLeftY() * maxSpeed; // Calculate the x speed based on the joystick input
        // final var ySpeed = -RobotContainer.controller.getLeftX() * maxSpeed; // Calculate the y speed based on the joystick input
        final var rot = swerveDrive.turnToAprilTag(9); // Calculate the rotation speed based on the joystick input
        swerveDrive.drive(0, 0, rot, true); // Drive the swerve drive
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
