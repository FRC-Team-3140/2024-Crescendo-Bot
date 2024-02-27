package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

//Works Well

public class ShooterSpeedL1 extends Command {
    private final IntakeShooter intakeShooter;
    private final double speed;
    private final double kShooterSpeedTolarence = 20;

    /**
     * Creates a new ShooterSpeedL1 command.
     *
     * @param intakeShooter The IntakeShooter subsystem
     * @param speed         The speed to set the IntakeShooter to
     */
    public ShooterSpeedL1(double speed) {
        this.intakeShooter = IntakeShooter.getInstance();
        this.speed = speed;
        addRequirements(intakeShooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * The command execution logic.
     * Sets the IntakeShooter to the desired speed.
     */
    @Override
    public void execute() {
        intakeShooter.setShooterSpeed(speed);
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
        return Math.abs(intakeShooter.getShooterSpeed() - speed) < kShooterSpeedTolarence;
    }
}