
package frc.robot;

import java.time.Instant;
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
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.SpitOutNote;
import frc.robot.commands.L2Commands.BasicSwerveControlL2;
import frc.robot.commands.L2Commands.SetArmToDistanceWhileMovingL2;
import frc.robot.commands.L3Commands.DriveFacingSpeaker;
// import frc.robot.commands.L3Commands.DriveFacingApril;
import frc.robot.commands.L3Commands.SpeakerShootDistanceL3;
import frc.robot.libs.XboxCotroller;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static XboxCotroller controller = new XboxCotroller(0);
  public static SwerveDrive swerve;
  public static Camera camera;
  public static Arm arm = Arm.getInstance();
  // private final Camera camera;
  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> autobuilder;
  public static Climber climber = Climber.getInstance();

  public static XboxCotroller controller2 = new XboxCotroller(1);
  public static IntakeShooter intakeShooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commppands.
   */
  public RobotContainer() {
    swerve = SwerveDrive.getInstance();
    camera = Camera.getInstance();
    swerve.setDefaultCommand(new BasicSwerveControlL2(swerve, maxChassisSpeed, maxChassisTurnSpeed));

    intakeShooter = IntakeShooter.getInstance();
    NamedCommands.registerCommand("IntakeUntilNoteDetected", new IntakeUntilNoteDetectedL1());

    NamedCommands.registerCommand("SpeakerShoot1",
        new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(3)));

    NamedCommands.registerCommand("SetArmToIntake", new SetArmToAngleL1(Arm.kSetpoiintIntakeDown));

    NamedCommands.registerCommand("SpeakerShoot2",
        new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(3)));

    NamedCommands.registerCommand("SpeakerShoot3",
        new ParallelCommandGroup(new SetArmToAngleL1(18), new ShootSpeakerL1(10., 3)));

    NamedCommands.registerCommand("StopMoving", new InstantCommand(()-> {swerve.drive(0, 0,0, false);}));

    NamedCommands.registerCommand("Wait", new WaitCommand(2));
    autobuilder = AutoBuilder.buildAutoChooser();

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

    new JoystickButton(controller, Button.kLeftBumper.value).onTrue(climber.increaseLeftHeight())
        .onFalse(new InstantCommand(climber::stopLeft));
    new JoystickButton(controller, Button.kRightBumper.value).onTrue(climber.increaseRightHeight())
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(rightTriggerC1).onTrue(new InstantCommand(climber::lowerRight))
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(leftTriggerC1).onTrue(new InstantCommand(climber::lowerLeft))
        .onFalse(new InstantCommand(climber::stopLeft));

    // Resetting Gyro
    new JoystickButton(controller, Button.kY.value).onTrue(new InstantCommand((swerve::resetGyro)));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = false;
    })).onFalse(new InstantCommand(() -> {
      BasicSwerveControlL2.fieldRelative = true;
    }));
    new JoystickButton(controller, Button.kA.value).whileTrue(new DriveFacingSpeaker(swerve, maxChassisSpeed));

    // Arm Controls
    new JoystickButton(controller2, Button.kY.value).onTrue(new SetArmToAngleL1(Arm.kSetpointAmp));
    new JoystickButton(controller2, Button.kB.value).onTrue(new SetArmToAngleL1(Arm.kSetpoiintIntakeDown));
    new JoystickButton(controller2, Button.kX.value).onTrue(new SetArmToAngleL1(Arm.kSetpointMove));
    // new JoystickButton(controller2, Button.kA.value).onTrue(new SpeakerShootDistanceL3()).onFalse(new ShooterSpeedL1(0));
    new JoystickButton(controller2, Button.kA.value).whileTrue(new RepeatCommand(new SetArmToDistanceL1()));
    new JoystickButton(controller2, Button.kStart.value).onTrue(new SetArmToAngleL1(16)).onTrue(new InstantCommand(()-> {swerve.resetPose(new Pose2d(1.3,5.5, new Rotation2d()));})); //Optimal angle for shooting from against the speaker.  
    new POVButton(controller2, 0).whileTrue(new RepeatCommand(new SetArmToDistanceWhileMovingL2()));
    //Intake/Shooter Controls     
    new JoystickButton(controller2, Button.kRightBumper.value).onTrue(new ShootAmpL1()).onFalse(new ShootSpeakerL1(0,0));
    new JoystickButton(controller2, Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new IntakeUntilNoteDetectedL1(), new SetArmToAngleL1(Arm.kSetpointMove)));
    BooleanSupplier rightTriggerC2 = () -> (controller2.getRightTriggerAxis() > 0.3);
    BooleanSupplier lefttTriggerC2 = () -> (controller2.getLeftTriggerAxis() > 0.3);
    new Trigger(lefttTriggerC2).onTrue(new ShootSpeakerL1(
      
    9.6,5)).onFalse(new ShootSpeakerL1(0, 0));
    new Trigger(rightTriggerC2).onTrue(new ShootSpeakerL1(9.6,0)).onFalse(new ShootSpeakerL1(0,0));
    new JoystickButton(controller2, Button.kBack.value).whileTrue(new SpitOutNote());

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