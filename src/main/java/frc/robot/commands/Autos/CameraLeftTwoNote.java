package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPathplannerPath;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;

/**
 * Represents a command group for executing a specific autonomous routine called
 * "CameraLeftTwoNote".
 * This routine involves picking up a note using the pickupNote command and
 * shooting the note using the CameraShootDistanceL3 command.
 */
public class CameraLeftTwoNote extends SequentialCommandGroup {
    // TODO: Test Camera Shoot distance!
    // static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

    /**
     * Constructs a new instance of the CameraLeftTwoNote command group.
     * Initializes the command group with the "CameraLeftTwoNote" autonomous routine
     * and the pickupNote and CameraShootDistanceL3 commands.
     */
    public CameraLeftTwoNote() {
        pickupNote intake = new pickupNote(false, RobotContainer.swerve, Camera.getInstance());
        SequentialCommandGroup shoot = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));
        SequentialCommandGroup shoot2 = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));

        addCommands(shoot, new FollowPathplannerPath("CameraLeftTwoNote", RobotContainer.swerve), intake,
                new FollowPathplannerPath("CameraLeftTwoNote2", RobotContainer.swerve),
                shoot2);
    }
}
