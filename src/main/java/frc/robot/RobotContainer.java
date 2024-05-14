
package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.pickupNote;
import frc.robot.commands.Autos.CameraRightTwoNote;
import frc.robot.commands.Autos.LeftThreeNote;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.OneNoteAuto;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.SetClimberToTopL1;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
import frc.robot.commands.L1Commands.SpitOutNote;
import frc.robot.commands.L1Commands.StopSpinningShooter;
import frc.robot.commands.L1Commands.ZeroClimbersL1;
import frc.robot.commands.L2Commands.BasicSwerveControlL2;
import frc.robot.commands.L3Commands.DriveFacingSpeaker;
import frc.robot.commands.L3Commands.SpeakerShootDistanceL3;
import frc.robot.libs.ControllerHelper;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // int leftBumperPresses = 0;
  pickupNote PickUpNoteCommand;

  public static ControllerHelper controller = new ControllerHelper(0);
  public static SwerveDrive swerve;
  public static Camera camera;
  public static Arm arm = Arm.getInstance();
  // private final Camera camera;
  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> autobuilder = new SendableChooser<>();
  public static Climber climber = Climber.getInstance();
  public static ControllerHelper controller2 = new ControllerHelper(1);
  public static Intake intake;
  public static Shooter shooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commppands.
   */
  public RobotContainer() {
    swerve = SwerveDrive.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    camera = Camera.getInstance();
    swerve
        .setDefaultCommand(new BasicSwerveControlL2(swerve, Constants.maxChassisSpeed, Constants.maxChassisTurnSpeed));
    shooter.setDefaultCommand(new StopSpinningShooter());
    PickUpNoteCommand = new pickupNote(true, swerve, camera);
    NamedCommands.registerCommand("IntakeUntilNoteDetected", new IntakeUntilNoteDetectedL1());

    NamedCommands.registerCommand("SpeakerShoot1",
        new SequentialCommandGroup(new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(2.5)),
            new ParallelRaceGroup(new ShootSpeakerOverrideL1(10, 3), new WaitCommand(2))));

    NamedCommands.registerCommand("SetArmToIntake", new SetArmToAngleL1(Arm.kSetpointIntakeDown));

    NamedCommands.registerCommand("SpeakerShoot2",
        new SequentialCommandGroup(new SetArmToDistanceL1(),
            new ParallelRaceGroup(new ShootSpeakerL1(10, 5), new WaitCommand(2.5)),
            new ParallelRaceGroup(new ShootSpeakerOverrideL1(10, 3), new WaitCommand(.3))));

    NamedCommands.registerCommand("SpeakerShoot3",
        new ParallelCommandGroup(new SetArmToAngleL1(16), new ShootSpeakerL1(6.5, 5).withTimeout(3)));

    NamedCommands.registerCommand("StopMoving", new InstantCommand(() -> {
      swerve.drive(0, 0, 0, false);
    }));
    NamedCommands.registerCommand("Turn To Speaker", new DriveFacingSpeaker(swerve, Constants.maxChassisSpeed));
    NamedCommands.registerCommand("Wait", new WaitCommand(2));

    // autobuilder = AutoBuilder.buildAutoChooser();
    autobuilder.addOption("One Note Auto", new OneNoteAuto());
    autobuilder.addOption("Left Three Note REAL", new LeftThreeNote());

    autobuilder.addOption("LeftTwoNote", AutoBuilder.buildAuto("LeftTwoNote"));
    autobuilder.addOption("LeftTwoNoteTwo", AutoBuilder.buildAuto("LeftTwoNoteTwo"));
    autobuilder.addOption("MiddleTwoNote", AutoBuilder.buildAuto("MiddleTwoNote"));
    autobuilder.addOption("MiddleTwoNoteTwo", AutoBuilder.buildAuto("MiddleTwoNoteTwo"));
    autobuilder.addOption("RightTwoNote", AutoBuilder.buildAuto("Far"));
    autobuilder.addOption("Camera Right", new CameraRightTwoNote());

    // // autobuilder.addOption("CameraLeftTwoNote", new CameraLeftTwoNote());
    // autobuilder.addOption("CameraMiddleTwoNote", new CameraMiddleTwoNote());
    // autobuilder.addOption("CameraRightTwoNote", new CameraRightTwoNote());
    // autobuilder.addOption("CameraLeftThreeNote", new CameraLeftThreeNote());

    // Additional Commands (Not automatically improted by Pathplanner) - TK
    // autobuilder.addOption("Pathfind To Apriltag", camera.pathfindToAprilTag());

    SmartDashboard.putData("Path planner", autobuilder);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Boolean suppliers
    BooleanSupplier rightTriggerC1 = () -> controller.getRightTriggerAxis() > .3;
    // BooleanSupplier leftTriggerC1 = () -> controller.getLeftTriggerAxis() > .3;

    BooleanSupplier rightTriggerC2 = () -> (controller2.getRightTriggerAxis() > 0.3);
    BooleanSupplier lefttTriggerC2 = () -> (controller2.getLeftTriggerAxis() > 0.3);

    BooleanSupplier upControllerLeftC2 = () -> (controller2.getLeftY() > 0.3);
    BooleanSupplier downControllerLeftC2 = () -> (controller2.getLeftY() < -0.3);
    // Primary Driver Controls

    // Resetting Gyro
    new JoystickButton(controller, Button.kY.value).onTrue(new InstantCommand((swerve::resetGyro)));
    new JoystickButton(controller, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> {if (!PickUpNoteCommand.isScheduled()) {PickUpNoteCommand.schedule();}}));
    new JoystickButton(controller, Button.kLeftBumper.value).onFalse(new InstantCommand(() -> PickUpNoteCommand.cancel()));
    // new JoystickButton(controller, Button.kA.value).whileTrue(new CameraShootDistanceL3());

    new Trigger(rightTriggerC1).onTrue(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = false;
    })).onFalse(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = true;
    }));

    // Secondary Driver Controls

    new JoystickButton(controller2, Button.kY.value).onTrue(new SetArmToAngleL1(Arm.kSetpointAmp));
    new JoystickButton(controller2, Button.kB.value).onTrue(new SetArmToAngleL1(Arm.kSetpointIntakeDown));
    new JoystickButton(controller2, Button.kX.value).onTrue(new SetArmToAngleL1(Arm.kSetpointMove));
    new JoystickButton(controller2, Button.kA.value).onTrue(new SetArmToAngleL1(16)).onTrue(new InstantCommand(() -> {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        swerve.resetPose(new Pose2d(1.3, 5.5, new Rotation2d()));
      } else {
        swerve.resetPose(new Pose2d(15.28, 5.58, new Rotation2d()));
      }
    })); // Optimal angle for shooting from against the speaker.
    new POVButton(controller2, 0).onTrue(new SetArmToAngleL1(33.75));
    new POVButton(controller2, 90).onTrue(new SetArmToAngleL1(35.5));

    new JoystickButton(controller2, Button.kStart.value).whileTrue(new RepeatCommand(new SetArmToDistanceL1()));
    new JoystickButton(controller2, Button.kRightBumper.value).onTrue(new ShootAmpL1())
        .onFalse(new ShootSpeakerL1(0, 0));// .onFalse(new ShootSpeakerL1(0,0));
    new JoystickButton(controller2, Button.kLeftBumper.value)
        .onTrue(new SequentialCommandGroup(new IntakeUntilNoteDetectedL1(), new SetArmToAngleL1(16)));

    new JoystickButton(controller2, Button.kBack.value).whileTrue(new SpitOutNote());

    new Trigger(lefttTriggerC2).whileTrue(new ShootSpeakerOverrideL1(Constants.shooterVoltage, 5))
        .onFalse(new ShootSpeakerL1(0, 0));

    new Trigger(upControllerLeftC2).onTrue(new ZeroClimbersL1());
    new Trigger(downControllerLeftC2).onTrue(new SetClimberToTopL1());

    new Trigger(rightTriggerC2).whileTrue(new ShootSpeakerOverrideL1(Constants.shooterVoltage, 0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autobuilder.getSelected();

  }

  // private void togglePickUpNote() {
  // leftBumperPresses++;
  // if (leftBumperPresses % 2 == 1) {
  // PickUpNoteCommand.schedule();
  // } else {
  // PickUpNoteCommand.cancel();
  // }
  // }

}

// TODO: When we get to comp, make sure to calibrate camera for apriltag and
// note detection. Test out Mr. Bolme's Turn to shoot code seperated. If that
// doesnt work, put it back to the way it was