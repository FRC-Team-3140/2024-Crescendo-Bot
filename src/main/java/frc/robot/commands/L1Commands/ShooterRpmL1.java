package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterRpmL1 extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final double rpm;

    /**
     * Creates a new ShooterSpeed command that sets the shooter to the specified
     * speed.
     *
     * @param speed the speed to set the shooter to
     */
    public ShooterRpmL1(double rpm) {
        this.rpm = rpm;
        addRequirements(shooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * Initializes the command.
     * Set the target speed when the command is initialized.
     */
    @Override
    public void execute() {
        shooter.setShooterRpm(rpm);
    }

    @Override
    public void end(boolean interrupted) {
        // shooter.stop();
    }

    /**
     * Checks if the shooter speed has reached the desired speed within a tolerance.
     * 
     * @return true if the shooter speed is within the tolerance, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}