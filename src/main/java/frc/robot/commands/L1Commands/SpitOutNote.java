package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//Works Well
/**
 * A command that sets the shooter to a specified speed using a PID controller.
 */
public class SpitOutNote extends Command {
    private final Intake intake;
    private final Shooter shooter;

    /**
     * Creates a new SpitOutNote command that sets the shooter and intake to the
     * specified speed.
     */
    public SpitOutNote() {
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        addRequirements(intake, shooter); // This command requires the IntakeShooter subsystem
    }

    /**
     * Initializes the command.
     * Set the target speed when the command is initialized.
     */
    @Override
    public void execute() {
        shooter.setShooterVoltage(-2);
        intake.setIntakeVoltage(-3);
    }

    /**
     * This method is called when the command ends.
     * It sets the shooter voltage and intake voltage to 0.
     * 
     * @param interrupted true if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVoltage(0);
        intake.setIntakeVoltage(0);
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