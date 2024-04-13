package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L3Commands.CameraShootDistanceL3;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

/**
 * Represents a command group for executing a specific autonomous routine called
 * "CameraLeftThreeNote".
 * This command group consists of a sequence of commands that includes picking
 * up a note, executing two other autonomous routines,
 * and shooting using a camera at a specific distance.
 */
public class CameraLeftThreeNote extends SequentialCommandGroup {
    static pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
    static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

    /**
     * Constructs a new instance of the CameraLeftThreeNote command.
     * This command initializes the CameraLeftTwoNote command as the first step,
     * followed by two custom autonomous routines,
     * and uses the intake and shoot subsystems for additional actions.
     */
    public CameraLeftThreeNote() {
        super(new CameraLeftTwoNote(), AutoBuilder.buildAuto("CameraLeftThreeNote1"), intake,
                AutoBuilder.buildAuto("CameraLeftThreeNote2"), shoot);
    }
}
