package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
//Working Good 
public class SetArmToAngleL1 extends Command {
    private final Arm arm;
    private final double targetAngle;
    private final double kDefaultError = .5; // in degrees

    /**
     * Creates a new SetArmToAngle command.
     *
     * @param targetAngle The target angle for the arm.
     * @param error The error for the arm.
     */
    public SetArmToAngleL1(double targetAngle) {
        this.arm = Arm.getInstance();
        this.targetAngle = targetAngle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Set the target angle when the command is initialized
        // arm.setAngle(targetAngle);
    }

    @Override
    public void execute() {
        // Nothing to do here, the arm's PIDController will handle moving the arm to the target angle
        arm.setAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        arm.resetVoltage();
        // Do nothing because the arm's PIDController will keep the arm at the target angle
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the arm is at the target angle
        // return true;
        return Math.abs(arm.getAngle() - targetAngle) < kDefaultError; 
    }
}