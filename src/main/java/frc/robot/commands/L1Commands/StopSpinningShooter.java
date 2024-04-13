package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * A command that stops the spinning of the shooter.
 */
public class StopSpinningShooter extends Command {

    /**
     * Creates a new StopSpinningShooter command.
     */
    public StopSpinningShooter() {
        addRequirements(Shooter.getInstance());
    }

    /**
     * Initializes the command.
     * Sets the shooter voltage to 0 when the command is initialized.
     */
    @Override
    public void initialize() {
        Shooter.getInstance().setShooterVoltage(0);
    }

    /**
     * Determines whether the command has finished executing.
     * 
     * @return true if the command has finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
