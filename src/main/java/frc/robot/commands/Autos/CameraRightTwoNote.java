package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

/**
 * Represents a command group for executing a specific autonomous routine called
 * "CameraRightTwoNote".
 * This routine involves setting the arm to a specific angle, shooting a
 * speaker, executing two custom autos,
 * and picking up a note using the camera.
 */
public class CameraRightTwoNote extends SequentialCommandGroup {
  public CameraRightTwoNote() {
    pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
    SequentialCommandGroup shoot = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
        new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));
    SequentialCommandGroup shoot2 = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
        new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));

    addCommands(shoot, AutoBuilder.buildAuto("CameraRightTwoNote1"), intake,
        AutoBuilder.buildAuto("CameraRightTwoNote2"), shoot2);
  }
}
