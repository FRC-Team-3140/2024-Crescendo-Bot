package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

//Works Well

/**
 * A command that sets the shooter to a specified speed using a PID controller.
 */
public class ShooterSpeedL1 extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final double speed;
    private final double kShooterSpeedTolarence = 1000;

    /**
     * Creates a new ShooterSpeed command that sets the shooter to the specified speed.
     *
     * @param speed the speed to set the shooter to
     */
    public ShooterSpeedL1(double speed) {
        this.speed = speed;
        addRequirements(shooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * Initializes the command.
     * Set the target speed when the command is initialized.
     */
    @Override
    public void execute() {
        shooter.setShooterSpeed(speed);
    }

    
    /**
     * Checks if the shooter speed has reached the desired speed within a tolerance.
     * @return true if the shooter speed is within the tolerance, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getShooterSpeed() - speed) < kShooterSpeedTolarence;
    }
}