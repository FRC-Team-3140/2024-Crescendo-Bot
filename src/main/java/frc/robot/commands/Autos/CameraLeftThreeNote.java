package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;

/**
 * Represents a command group for executing a specific autonomous routine called
 * "CameraLeftThreeNote".
 * This command group consists of a sequence of commands that includes picking
 * up a note, executing two other autonomous routines,
 * and shooting using a camera at a specific distance.
 */
public class CameraLeftThreeNote extends SequentialCommandGroup {
    // TODO: Test Camera Shoot distance!
    // static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

    /**
     * Constructs a new instance of the CameraLeftThreeNote command.
     * This command initializes the CameraLeftTwoNote command as the first step,
     * followed by two custom autonomous routines,
     * and uses the intake and shoot subsystems for additional actions.
     */
    public CameraLeftThreeNote() {
        pickupNote intake2 = new pickupNote(false, RobotContainer.swerve, Camera.getInstance());
        SequentialCommandGroup shoot3 = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));
                        /*.andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage)));*/

        addCommands(new CameraLeftTwoNote(),
                AutoBuilder.buildAuto("CameraLeftThreeNote1"), intake2,
                AutoBuilder.buildAuto("CameraLeftThreeNote2"), shoot3);
    }
}
