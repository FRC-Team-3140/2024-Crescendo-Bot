package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmToAngle extends Command {
    private final Arm arm;
    private final double targetAngle;
    private final double kDefaultError = 2.0; // in degrees

    /**
     * Creates a new SetArmToAngle command.
     *
     * @param targetAngle The target angle for the arm.
     * @param error The error for the arm.
     */
    public SetArmToAngle(double targetAngle) {
        this.arm = Arm.getInstance();
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Set the target angle when the command is initialized
        arm.setAngle(targetAngle);
    }

    @Override
    public void execute() {
        // Nothing to do here, the arm's PIDController will handle moving the arm to the target angle
    }

    @Override
    public void end(boolean interrupted) {
        // Do nothing because the arm's PIDController will keep the arm at the target angle
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the arm is at the target angle
        return Math.abs(arm.getAngle() - targetAngle) < kDefaultError; 
    }
}