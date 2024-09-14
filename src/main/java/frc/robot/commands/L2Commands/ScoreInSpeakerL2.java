package frc.robot.commands.L2Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShooterSpeedL1;
import frc.robot.subsystems.Arm;

// TODO: This class is not used.
/**
 * This class represents a command to score in the speaker.
 * It extends SequentialCommandGroup to perform a sequence of commands.
 */
public class ScoreInSpeakerL2 extends SequentialCommandGroup {
        static private final double kArmAngle = Arm.kSetpointShoot; // The desired arm angle in degrees
        static private final double kShooterSpeed = 0.9; // The desired shooter speed as a fraction of max speed

        /**
         * Creates a new ScoreInSpeakerL2 command.
         *
         * @param arm           The Arm subsystem
         * @param intakeShooter The IntakeShooter subsystem
         */
        public ScoreInSpeakerL2() {
                addCommands(

                                // Step 1: In parallel, set the arm to an angle and rev the shooter speed so it
                                // is ready to fire.
                                new ParallelCommandGroup(
                                                // Set the arm to the desired angle
                                                new SetArmToAngleL1(kArmAngle),
                                                // Set the shooter to the desired speed
                                                new ShooterSpeedL1(kShooterSpeed)),

                                new PrintCommand("Reached Speed!"),

                                // Step 2: When both are ready, shoot at the speaker.
                                new ShootSpeakerL1(10, 3),

                                // Step 3: When done, stop the shooter and reset the arm to move position.
                                new ParallelCommandGroup(
                                                // Reset the arm to the move position
                                                new SetArmToAngleL1(Arm.kSetpointMove)));
        }
}