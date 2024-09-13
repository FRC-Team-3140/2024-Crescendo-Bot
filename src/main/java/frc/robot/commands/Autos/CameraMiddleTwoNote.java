package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;

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
        SequentialCommandGroup shoot = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3)
                        .andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage)));
        SequentialCommandGroup shoot2 = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3)
                        .andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage)));

        addCommands(shoot, AutoBuilder.buildAuto("CameraMiddleTwoNote"), intake, shoot2);
    }

}
