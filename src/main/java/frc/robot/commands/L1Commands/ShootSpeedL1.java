package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//Works Well

/**
 * A command that sets the shooter to a specified speed using a PID controller.
 */
public class ShootSpeedL1 extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final double rpm;
    private final double kShooterSpeedTolarence = 3;
    private int counter = 0;

    /**
     * Creates a new ShooterSpeed command that sets the shooter to the specified
     * speed.
     *
     * @param rpm the speed to set the shooter to
     */
    public ShootSpeedL1(double rpm) {
        this.rpm = rpm;
        addRequirements(intake, shooter); // This command requires the IntakeShooter subsystem
    }

    @Override
    public void initialize() {
        counter = 0;
    }

    /**
     * Initializes the command.
     * Set the target speed when the command is initialized.
     */
    @Override
    public void execute() {
        shooter.setShooterRpm(rpm);

        // System.out.println(counter);

        if (Math.abs(shooter.getShooterSpeed() - rpm) < kShooterSpeedTolarence) {
            intake.setIntakeVoltage(Constants.intakeVoltage);

            counter++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVoltage(0);
        shooter.setShooterRpm(0);
    }

    /**
     * Checks if the shooter speed has reached the desired speed within a tolerance.
     * 
     * @return true if the shooter speed is within the tolerance, false otherwise.
     */
    @Override
    public boolean isFinished() {
        // return Math.abs(shooter.getShooterSpeed() - rpm) < kShooterSpeedTolarence;
        return counter >= 60;
    }
}