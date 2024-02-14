package frc.robot.commands.L2Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShooterSpeedL1;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;


// TODO: This needs testing - DB
/**
 * This class represents a command to score in the amp.
 * It extends SequentialCommandGroup to perform a sequence of commands.
 */
public class ScoreInAmpL2 extends SequentialCommandGroup {
    static private final double kArmAngle = 94.0; // The desired arm angle in degrees
    static private final double kShooterSpeed = 0.2; // The desired shooter speed as a fraction of max speed

    /**
     * Creates a new ScoreInSpeakerL2 command.
     *
     * @param arm The Arm subsystem
     * @param intakeShooter The IntakeShooter subsystem
     */
    public ScoreInAmpL2(Arm arm, IntakeShooter intakeShooter) {
        addCommands(

            // Step 1: In parallel, set the arm to an angle and rev the shooter speed so it is ready to fire.
            new ParallelCommandGroup(
                // Set the arm to the desired angle
                new SetArmToAngleL1( kArmAngle),
                // Set the shooter to the desired speed
                new ShooterSpeedL1( kShooterSpeed)
            ),

            // Step 2: When both are ready, shoot at the speaker.
            new ShootAmpL1(),

            // Step 3: When done, stop the shooter and reset the arm to move position.
            new ParallelCommandGroup(
                // Reset the arm to the move position
                new SetArmToAngleL1( Arm.kSetpointMove )
            )
        );
    }
}