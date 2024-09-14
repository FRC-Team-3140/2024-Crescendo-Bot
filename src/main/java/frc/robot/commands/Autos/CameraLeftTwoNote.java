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
 * "CameraLeftTwoNote".
 * This routine involves picking up a note using the pickupNote command and
 * shooting the note using the CameraShootDistanceL3 command.
 */
public class CameraLeftTwoNote extends SequentialCommandGroup {
        /**
         * Constructs a new instance of the CameraLeftTwoNote command group.
         * Initializes the command group with the "CameraLeftTwoNote" autonomous routine
         * and the pickupNote and CameraShootDistanceL3 commands.
         */
        public CameraLeftTwoNote() {
                pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
                SequentialCommandGroup shoot = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));
                                                /*.andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage)));*/
                SequentialCommandGroup shoot2 = new SequentialCommandGroup(new SetArmToAngleL1(Arm.kSetpointShoot),
                                new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage).withTimeout(3));
                                /*.andThen(new ShootSpeakerOverrideL1(1, Constants.intakeVoltage));*/
                // TODO: Test Camera Shoot distance!
                // static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();

                addCommands(shoot, AutoBuilder.buildAuto("CameraLeftTwoNote"), intake,
                                AutoBuilder.buildAuto("CameraLeftTwoNote2"), shoot2);

                // TODO: Most likely can delete this test code after RoboRodeo 2024!
                // addCommands(shoot, new PrintCommand("Path"),
                // path1,
                // new InstantCommand(
                // () -> {
                // path1.end(true);
                // addRequirements(swerve);
                // swerve
                // .setModuleStates(new SwerveModuleState[] {
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)) });
                // }),
                // new PrintCommand("intake"), intake,
                // path2,
                // new InstantCommand(
                // () -> {
                // path2.end(true);
                // addRequirements(swerve);
                // System.out.println("Resetting");
                // swerve
                // .setModuleStates(new SwerveModuleState[] {
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)),
                // new SwerveModuleState(0,
                // new Rotation2d(0)) });
                // }),
                // shoot2);

                // addCommands(new InstantCommand(() ->
                // SwerveDrive.getInstance().setModuleStates(new SwerveModuleState[] {
                // new SwerveModuleState(1,
                // new Rotation2d(0)),
                // new SwerveModuleState(1,
                // new Rotation2d(0)),
                // new SwerveModuleState(1,
                // new Rotation2d(0)),
                // new SwerveModuleState(1,
                // new Rotation2d(0)) })));
        }
}
