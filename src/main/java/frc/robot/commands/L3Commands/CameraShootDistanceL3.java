
package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.L1Commands.DetectAprilTagL1;
import frc.robot.commands.L1Commands.SetArmToSpeakerDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
import frc.robot.commands.L1Commands.TurnBotToSpeakerL1;

public class CameraShootDistanceL3 extends SequentialCommandGroup {

    public CameraShootDistanceL3() {
        // TODO: Currently this probabably wont work if the april tag detection fails.  That case needs to be handled and tested.
        DetectAprilTagL1 detectAprilTag = new DetectAprilTagL1(1.0);

        ParallelCommandGroup setupShot = new ParallelCommandGroup(
                new TurnBotToSpeakerL1(detectAprilTag),
                new SetArmToSpeakerDistanceL1(detectAprilTag));

        // ShootSpeakerOverrideL1 shootSpeaker = new ShootSpeakerOverrideL1(9.6, 5);

        this.addCommands(detectAprilTag, setupShot);
    }

}
