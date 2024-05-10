// package frc.robot.commands.Autos;

// import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.pickupNote;
// import frc.robot.commands.L3Commands.CameraShootDistanceL3;
// import frc.robot.sensors.Camera;
// import frc.robot.subsystems.SwerveDrive;

// /**
//  * Represents a command group for executing a specific autonomous routine called
//  * "CameraLeftTwoNote".
//  * This routine involves picking up a note using the pickupNote command and
//  * shooting the note using the CameraShootDistanceL3 command.
//  */
// public class CameraLeftTwoNote extends SequentialCommandGroup {
//     static pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
//     static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

//     /**
//      * Constructs a new instance of the CameraLeftTwoNote command group.
//      * Initializes the command group with the "CameraLeftTwoNote" autonomous routine
//      * and the pickupNote and CameraShootDistanceL3 commands.
//      */
//     public CameraLeftTwoNote() {
//         super(AutoBuilder.buildAuto("CameraLeftTwoNote"), intake, shoot);
//     }
// }
