package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L2Commands.ScoreInSpeakerL2;
import frc.robot.sensors.Camera;

/**
 * Represents a command group for executing the autonomous routine
 * "CameraMiddleTwoNote".
 * This command group includes the pickupNote command and the
 * CameraShootDistanceL3 command.
 */
public class CameraMiddleTwoNote extends SequentialCommandGroup {
    // static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

    public CameraMiddleTwoNote() {
        pickupNote intake = new pickupNote(false, RobotContainer.swerve, Camera.getInstance());
        ScoreInSpeakerL2 shoot = new ScoreInSpeakerL2();
        ScoreInSpeakerL2 shoot2 = new ScoreInSpeakerL2();

        addCommands(shoot, AutoBuilder.buildAuto("CameraMiddleTwoNote"), intake, shoot2);
    }

}
