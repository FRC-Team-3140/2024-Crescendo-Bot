package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPathplannerPath;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;

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

    addCommands(new SetArmToAngleL1(Arm.kSetpointShoot), new ShootSpeakerL1(6.5, 5).withTimeout(3),
        new FollowPathplannerPath("CameraRightTwoNote1", RobotContainer.swerve), intake,
        new FollowPathplannerPath("CameraRightTwoNote2", RobotContainer.swerve));
  }
}
