package frc.robot.commands.L1Commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

/**
 * This command sets the arm to a specific distance.
 * The distance is determined by the distance from the speaker.
 * The command is finished when the arm is at the target angle within a tolerance.
 */
public class SetArmToDistanceL1 extends Command {
    private final Arm arm = Arm.getInstance();
    private double distance;
    private double angleTolerance = .5;

    /**
     * Creates a new SetArmToDistance command that moves the arm to the specified distance.
     */
    public SetArmToDistanceL1() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // distance = Math.hypot(SwerveDrive.getInstance().getPose().getX(),
        // SwerveDrive.getInstance().getPose().getY()- (216*.0254));
        // SmartDashboard.putNumber("Distance", distance);
        // Code to initialize the command arm.setArmToShootDistance(distance);
    }

    /**
        * Executes the command.
        * Retrieves the distance from the speaker of the Swerve Drive,
        * displays it on the SmartDashboard, and sets the arm to shoot
        * at the calculated distance.
        */
    @Override
    public void execute() {
        // Code to initialize the command
        distance = SwerveDrive.getInstance().getDistanceFromSpeaker(); // TODO: I am concerned this comes from the swerve drive.
        SmartDashboard.putNumber("Distance", distance);
        arm.setArmToShootDistance(distance);
    }

    /**
     * Checks if the command is finished.
     * The command is considered finished when the arm is at the target angle within the specified angle tolerance.
     *
     * @return true if the command is finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        // The command is finished when the arm is at the target angle
        return Math.abs(NetworkTableInstance.getDefault().getTable("Arm").getEntry("Setpoint").getDouble(8)
                - arm.getAngle()) < angleTolerance;
    }

    /**
        * This method is called when the command ends.
        * 
        * @param interrupted true if the command was interrupted, false otherwise
        */
    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends
    }
}