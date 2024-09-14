package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L2Commands.ScoreInSpeakerL2;
import frc.robot.sensors.Camera;

/**
 * Represents a command group for executing a specific autonomous routine called
 * "CameraRightTwoNote".
 * This routine involves setting the arm to a specific angle, shooting a
 * speaker, executing two custom autos,
 * and picking up a note using the camera.
 */
public class CameraRightTwoNote extends SequentialCommandGroup {
  public CameraRightTwoNote() {
    pickupNote intake = new pickupNote(false, RobotContainer.swerve, Camera.getInstance());
    ScoreInSpeakerL2 shoot = new ScoreInSpeakerL2();
    ScoreInSpeakerL2 shoot2 = new ScoreInSpeakerL2();

    addCommands(shoot, AutoBuilder.buildAuto("CameraRightTwoNote1"), intake,
        AutoBuilder.buildAuto("CameraRightTwoNote2"), shoot2);
  }
}
