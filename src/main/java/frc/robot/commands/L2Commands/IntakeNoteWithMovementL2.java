package frc.robot.commands.L2Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;

// TODO: Test this carefully. This is a complex command that involves multiple subsystems.
/**
 * This command class represents the intake process with movement for the L2 level.
 * It sets the arm to a specific angle and intake speed in parallel, then moves the robot backward
 * when both are ready. Once the intake has the note, it stops the intake, stops the drivetrain,
 * and raises the arm to a desired angle.
 */
public class IntakeNoteWithMovementL2 extends Command {
    private Arm arm = Arm.getInstance();
    private IntakeShooter intakeShooter = IntakeShooter.getInstance();
    private SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public IntakeNoteWithMovementL2() {
        addRequirements(arm, intakeShooter, swerveDrive);
    }

    @Override
    public void initialize() {
        // Step 1: In parallel, set the arm to an angle and intake speed.
        arm.setAngle(Arm.kSetpoiintIntakeDown); // Assuming 0.0 is the desired angle
        intakeShooter.setIntakeVoltage(Constants.intakeVoltage); // Assuming 1.0 is the desired speed
    }

    @Override
    public void execute() {
        // Step 2: When both are ready, move the robot backward.
        boolean armIsReady = Math.abs( arm.getAngle() - Arm.kSetpoiintIntakeDown) < 1.0; // Assuming 0.1 is the desired tolerance
        if (armIsReady) {
            swerveDrive.drive(0.5, 0.0, 0.0,false); // Assuming 0.5 is the desired speed to move backward
        }

        // Finished: When the intake has the note.
        if (intakeShooter.hasNote()) {
            // End: stop the intake. Stop the drivetrain, and raise the arm.
            // TODO: intakeShooter.stop();
            swerveDrive.drive(0,0,0,false);
            arm.setAngle(Arm.kSetpointIntakeReady); // Assuming 90.0 is the desired angle to raise the arm
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: intakeShooter.stop(); 
        swerveDrive.drive(0,0,0,false);
        arm.setAngle(Arm.kSetpointIntakeReady); // If intake failed this will be a good place for the arm to try again.
    }

    @Override
    public boolean isFinished() {
        return intakeShooter.hasNote();
    }
}