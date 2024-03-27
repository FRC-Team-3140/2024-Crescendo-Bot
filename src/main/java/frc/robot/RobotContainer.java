
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
import frc.robot.commands.L1Commands.DetectAprilTagL1;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.OneNoteAuto;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.SetClimberToPosition;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
import frc.robot.commands.L1Commands.SpitOutNote;
import frc.robot.commands.L1Commands.TurnBotToAngleL1;
import frc.robot.commands.L2Commands.BasicSwerveControlL2;
import frc.robot.commands.L2Commands.DetectTagAndTurnL2;
import frc.robot.commands.L2Commands.SetArmToDistanceWhileMovingL2;
import frc.robot.commands.L3Commands.DriveFacingSpeaker;
// import frc.robot.commands.L3Commands.DriveFacingApril;
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
import edu.wpi.first.networktables.NetworkTableInstance;
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
public class RobotContainer implements Constants {
  public static ControllerHelper controller = new ControllerHelper(0);
  public static SwerveDrive swerve;
  public static Camera camera;
  public static Arm arm = Arm.getInstance();
  // private final Camera camera;
  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> autobuilder;
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
    swerve.setDefaultCommand(new BasicSwerveControlL2(swerve, maxChassisSpeed, maxChassisTurnSpeed));

    NamedCommands.registerCommand("IntakeUntilNoteDetected", new IntakeUntilNoteDetectedL1());

    NamedCommands.registerCommand("SpeakerShoot1",
        new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(5)));

    NamedCommands.registerCommand("SetArmToIntake", new SetArmToAngleL1(Arm.kSetpoiintIntakeDown));

    NamedCommands.registerCommand("SpeakerShoot2",
        new SequentialCommandGroup(new SetArmToDistanceL1(), new ShootSpeakerL1(10, 3), new WaitCommand(5),new ShootSpeakerL1(0, 0)));

    NamedCommands.registerCommand("SpeakerShoot3",
        new ParallelCommandGroup(new SetArmToAngleL1(18), new ShootSpeakerL1(10., 5)));

    NamedCommands.registerCommand("StopMoving", new InstantCommand(()-> {swerve.drive(0, 0,0, false);}));

    NamedCommands.registerCommand("Wait", new WaitCommand(2));
    
    
    autobuilder = AutoBuilder.buildAutoChooser();
    autobuilder.addOption("One Note Auto", new OneNoteAuto());

    // Additional Commands (Not automatically improted by Pathplanner) - TK
    // autobuilder.addOption("Pathfind To Apriltag", camera.pathfindToAprilTag());

    SmartDashboard.putData("Path planner", autobuilder);

    NetworkTableInstance.getDefault().getTable("Double").getEntry("Test").setDouble(2);
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
    BooleanSupplier rightTriggerC1 = () -> controller.getRightTriggerAxis() > .3;
    BooleanSupplier leftTriggerC1 = () -> controller.getLeftTriggerAxis() > .3;

    // Resetting Gyro
    new JoystickButton(controller, Button.kY.value).onTrue(new InstantCommand((swerve::resetGyro)));
    new Trigger(rightTriggerC1).onTrue(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = false;
    })).onFalse(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = true;
    }));
    new JoystickButton(controller, Button.kA.value).whileTrue(new DriveFacingSpeaker(swerve, maxChassisSpeed));
    controller.setRumble();
    // Arm Controls
    new JoystickButton(controller2, Button.kY.value).onTrue(new SetArmToAngleL1(Arm.kSetpointAmp));
    new JoystickButton(controller2, Button.kB.value).onTrue(new SetArmToAngleL1(Arm.kSetpoiintIntakeDown));
    new JoystickButton(controller2, Button.kX.value).onTrue(new SetArmToAngleL1(Arm.kSetpointMove));
    new POVButton(controller2, 0).onTrue(new SetArmToAngleL1(35.5));
    // new JoystickButton(controller2, Button.kA.value).onTrue(new SpeakerShootDistanceL3()).onFalse(new ShooterSpeedL1(0));
    new JoystickButton(controller2, Button.kStart.value).whileTrue(new RepeatCommand(new SetArmToDistanceL1()));
    new JoystickButton(controller2, Button.kA.value).onTrue(new SetArmToAngleL1(16)).onTrue(new InstantCommand(() -> {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        swerve.resetPose(new Pose2d(1.3, 5.5, new Rotation2d()));
      } else {
        swerve.resetPose(new Pose2d(15.28, 5.58, new Rotation2d()));
      }
    }));

    // TODO: Testing distance shoot.  Delete after testing.
    //new JoystickButton(controller2, Button.kBack.value).whileTrue(new TurnBotToAngleL1(45.0).withTimeout(5));
    //new JoystickButton(controller2, Button.kBack.value).whileTrue(new DetectAprilTagL1(10).withTimeout(5));
    new JoystickButton(controller2, Button.kBack.value).whileTrue(new DetectTagAndTurnL2().withTimeout(5));
    

    //Optimal angle for shooting from against the speaker.  
    // new POVButton(controller2, 0).whileTrue(new RepeatCommand(new SetArmToDistanceWhileMovingL2()));
    //Intake/Shooter Controls     
    new JoystickButton(controller2, Button.kRightBumper.value).onTrue(new ShootAmpL1()).onFalse(new ShootSpeakerL1(0, 0));//.onFalse(new ShootSpeakerL1(0,0));
    new JoystickButton(controller2, Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new IntakeUntilNoteDetectedL1(), new SetArmToAngleL1(16)));
    BooleanSupplier rightTriggerC2 = () -> (controller2.getRightTriggerAxis() > 0.3);
    BooleanSupplier lefttTriggerC2 = () -> (controller2.getLeftTriggerAxis() > 0.3);
    new Trigger(lefttTriggerC2).onTrue(new ShootSpeakerOverrideL1(9.6,5)).onFalse(new ShootSpeakerOverrideL1(0, 0));//.onFalse(new ShootSpeakerL1(0, 0));
    BooleanSupplier upControllerLeftC2 = () -> (controller2.getLeftY() > 0.3);
    BooleanSupplier downControllerLeftC2 = () -> (controller2.getLeftY() < -0.3);
    BooleanSupplier upControllerRightC2 = () -> (controller2.getRightY() > 0.3);
    BooleanSupplier downControllerRightC2 = () -> (controller2.getRightY() < -0.3);
    
    new Trigger(upControllerLeftC2).onTrue(new SetClimberToPosition(topClimberPosition))
        .onFalse(new InstantCommand(climber::stopLeft));
    new Trigger(upControllerRightC2).onTrue(new SetClimberToPosition(0))
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(downControllerRightC2).onTrue(new InstantCommand(climber::lowerRight))
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(downControllerLeftC2).onTrue(new InstantCommand(climber::lowerLeft))
        .onFalse(new InstantCommand(climber::stopLeft));

    new Trigger(rightTriggerC2).onTrue(new ShootSpeakerOverrideL1(9.6,0)).onFalse(new ShootSpeakerOverrideL1(0,0));
    //new JoystickButton(controller2, Button.kBack.value).whileTrue(new SpitOutNote());

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

}