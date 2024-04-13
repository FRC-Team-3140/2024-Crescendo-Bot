package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * A command that sets the arm to a specified angle using a PID controller.
 */
public class SetArmToAngleL1 extends Command {
    private final Arm arm;
    private final double targetAngle;
    private final double kDefaultError = .5; // in degrees

    /**
     * Creates a new SetArmToAngle command that moves the arm to the specified angle.
     *
     * @param targetAngle the angle to move the arm to
     */
    public SetArmToAngleL1(double targetAngle) {
        this.arm = Arm.getInstance();
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    
    /**
        * Initializes the command.
        * Set the target angle when the command is initialized.
        */
    @Override
    public void initialize() {
        // Set the target angle when the command is initialized
        // arm.setAngle(targetAngle);
    }

    /**
        * Executes the command.
        * This method sets the target angle for the arm and lets the arm's PIDController handle
        * moving the arm to the target angle.
        */
    @Override
    public void execute() {
        // Nothing to do here, the arm's PIDController will handle moving the arm to the
        // target angle
        arm.setAngle(targetAngle);
    }

    /**
     * This method is called when the command ends. It resets the voltage of the arm and does nothing else,
     * as the arm's PIDController will keep the arm at the target angle.
     * 
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        arm.resetVoltage();
        // Do nothing because the arm's PIDController will keep the arm at the target
        // angle
    }

    /**
     * Checks if the command is finished.
     * The command is considered finished when the arm is at the target angle.
     * @return true if the command is finished, false otherwise.
     */
    @Override
    public boolean isFinished() {
        // The command is finished when the arm is at the target angle
        // return true;
        return Math.abs(arm.getAngle() - targetAngle) < kDefaultError;
    }
}