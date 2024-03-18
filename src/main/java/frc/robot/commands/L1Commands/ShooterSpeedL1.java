package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

//Works Well

public class ShooterSpeedL1 extends Command {
    private final Shooter shooter;
    private final double speed;
    private final double kShooterSpeedTolarence = 20;

    /**
     * Creates a new ShooterSpeedL1 command.
     *
     * @param intakeShooter The IntakeShooter subsystem
     * @param speed         The speed to set the IntakeShooter to
     */
    public ShooterSpeedL1(double speed) {
        this.shooter = Shooter.getInstance();
        this.speed = speed;
        addRequirements(shooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * The command execution logic.
     * Sets the IntakeShooter to the desired speed.
     */
    @Override
    public void execute() {
        shooter.setShooterSpeed(speed);
    }

    /**
     * Determines whether the command is finished.
     * If this command is the default command for the shooter, it should never
     * finish.
     *
     * @return false because this command should never finish if it's the default
     *         command for the shooter
     */

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getShooterSpeed() - speed) < kShooterSpeedTolarence;
    }
}