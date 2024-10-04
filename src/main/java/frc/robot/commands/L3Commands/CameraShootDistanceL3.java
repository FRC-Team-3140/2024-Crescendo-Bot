
package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.L1Commands.DetectAprilTagL1;
import frc.robot.commands.L1Commands.SetArmToSpeakerDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.TurnBotToSpeakerL1;

/**
 * Represents a command group for shooting at a specific distance using camera
 * detection.
 */
public class CameraShootDistanceL3 extends SequentialCommandGroup {
    ParallelCommandGroup setupShot = new ParallelCommandGroup();

    /**
     * Creates a new CameraShootDistanceL3 command.
     */
    public CameraShootDistanceL3() {
        // TODO: Currently this probabably wont work if the april tag detection fails.
        // That case needs to be handled and tested.

        DetectAprilTagL1 detectAprilTag = new DetectAprilTagL1(1.0);
        setupShot = new ParallelCommandGroup(
                new TurnBotToSpeakerL1(detectAprilTag).withTimeout(2),
                new SetArmToSpeakerDistanceL1(detectAprilTag),
                new ShootSpeakerL1(11, 5));

        this.addCommands(detectAprilTag, setupShot);
    }

}
