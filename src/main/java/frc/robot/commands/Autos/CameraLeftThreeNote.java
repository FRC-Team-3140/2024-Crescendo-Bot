package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L2Commands.ScoreInSpeakerL2;
import frc.robot.RobotContainer;
import frc.robot.sensors.Camera;

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
        ScoreInSpeakerL2 shoot3 = new ScoreInSpeakerL2();
        /* .andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage))); */
        Command path1 = AutoBuilder.buildAuto("CameraLeftThreeNote1");
        Command path2 = AutoBuilder.buildAuto("CameraLeftThreeNote2");

        addCommands(new CameraLeftTwoNote(), path1, intake2, path2, shoot3);
    }
}
