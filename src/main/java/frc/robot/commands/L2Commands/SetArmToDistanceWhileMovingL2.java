package frc.robot.commands.L2Commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

// TODO: This class is not used.
/**
 * This class represents a command to update the arm position while moving based on the distace to the speaker.
 * It extends Command to be used as a command in the command-based programming.
 */
public class SetArmToDistanceWhileMovingL2 extends Command {
    private final Arm arm = Arm.getInstance();
    private double distance;
    // private double setpoint;
    private double angleTolerance = .1;

    /**
     * Creates a new SetArmToDistanceWhileMovingL2 command.
     */
    public SetArmToDistanceWhileMovingL2() {
        // this.distance = distance;
        addRequirements(arm);
    }

    /**
        * Initializes the command.
        * This method is called once when the command is first scheduled.
        * It calculates the distance and sets the arm to shoot distance.
        */
    @Override
    public void initialize() {
        // distance = Math.hypot(SwerveDrive.getInstance().getPose().getX(),
        // SwerveDrive.getInstance().getPose().getY()- (216*.0254));
        // SmartDashboard.putNumber("Distance", distance);
        // Code to initialize the command arm.setArmToShootDistance(distance);
    }

    /**
        * Executes the command.
        * This method initializes the command, retrieves the expected distance from the speaker,
        * and sets the arm to the shoot distance.
        */
    @Override
    public void execute() {
        // Code to initialize the command
        distance = SwerveDrive.getInstance().getExpectedDistanceFromSpeaker();
        SmartDashboard.putNumber("Distance", distance);
        arm.setArmToShootDistance(distance);
    }

    /**
     * Determines whether the command is finished or not.
     * The command is considered finished when the arm is at the target angle within the specified tolerance.
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