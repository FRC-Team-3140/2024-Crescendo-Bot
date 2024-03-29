package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//Works Well

public class SpitOutNote extends Command {
    private final Intake intake;
    private final Shooter shooter;

    /**
     * Creates a new ShooterSpeedL1 command.
     *
     * @param intakeShooter The IntakeShooter subsystem
     * @param speed         The speed to set the IntakeShooter to
     */
    public SpitOutNote() {
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        addRequirements(intake, shooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * The command execution logic.
     * Sets the IntakeShooter to the desired speed.
     */
    @Override
    public void execute() {
        shooter.setShooterVoltage(-2);
        intake.setIntakeVoltage(-3);
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
    public void end(boolean interrupted) {
        shooter.setShooterVoltage(0);
        intake.setIntakeVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}