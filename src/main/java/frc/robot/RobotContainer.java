
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L2Commands.BasicSwerveControlL2;
import frc.robot.commands.L3Commands.SpeakerShootDistanceL3;
import frc.robot.libs.XboxCotroller;
import frc.robot.sensors.Camera;
// // import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
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
public class RobotContainer implements Constants{
  public static XboxCotroller controller = new XboxCotroller(0);
  public static AHRS gyro = new AHRS(Port.kMXP);
  public static SwerveDrive swerve = SwerveDrive.getInstance();
  public static Camera camera = Camera.getInstance();
  // // private final Camera camera;
  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> autobuilder;
  

  public static XboxCotroller controller2 = new XboxCotroller(1);
  public static IntakeShooter intakeShooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerve.setDefaultCommand(new BasicSwerveControlL2(swerve, maxSpeed, maxChassisTurnSpeed));
    intakeShooter = IntakeShooter.getInstance();
    NamedCommands.registerCommand("IntakeUntilNoteDetected", new IntakeUntilNoteDetectedL1());
    NamedCommands.registerCommand("SpeakerShoot", new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(1)));
    
    // Additional Commands (Not automatically improted by Pathplanner) - TK
    autobuilder.addOption("Pathfind To AprilTag", camera.pathfindToAprilTag());
    
    autobuilder = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Path planner", autobuilder);
  
    // camera = Camera.getInstance();
    // autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    // autoChooser.addOption("Auto2", new PathPlannerAuto("Auto2"));
    // autoChooser.addOption("Auto3", new PathPlannerAuto("Auto3"));

    // SmartDashboard.putData("Auto", autoChooser);
    // Configure the trigger bindings

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
    // new JoystickButton(controller2, Button.kA.value).onTrue(new SetArmToAngle(8));
    new JoystickButton(controller2, Button.kB.value).onTrue(new SetArmToAngleL1(Arm.kSetpoiintIntakeDown));
    new JoystickButton(controller2, Button.kY.value).onTrue(new SetArmToAngleL1(Arm.kSetpointAmp));
    new JoystickButton(controller2, Button.kX.value).onTrue(new SetArmToAngleL1(Arm.kSetpointMove));
    new JoystickButton(controller2, Button.kA.value).onTrue(new SpeakerShootDistanceL3()).onFalse(new ShootSpeakerL1(0, 0));


    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand((this::resetGyro)));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(()-> swerve.resetPose(new Pose2d(.39, 7.8, new Rotation2d())))); //top left corner
    new JoystickButton(controller, Button.kX.value).onTrue(new InstantCommand(()-> swerve.resetPose(new Pose2d(1.2, 5.56, new Rotation2d())))); //right in front of speaker
    
    // new POVButton(controller2, 0).onTrue(new SpeakerShoot()).onFalse(new InstantCommand(()-> {intakeShooter.setShooterVoltage(0);}));
    new POVButton(controller2, 90).onTrue(new ShootAmpL1()).onFalse(new ShootSpeakerL1(0,0));
    
    new POVButton(controller2, 180).onTrue(new IntakeUntilNoteDetectedL1());
    new POVButton(controller2, 0).onTrue(new InstantCommand(()-> {intakeShooter.setHoldingPiece(true);}));
    new POVButton(controller2, 270).onTrue(new ShootSpeakerL1(10,3)).onFalse(new ShootSpeakerL1(0,0));
    
  } 

  public void resetGyro(){
    gyro.reset();
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
    // return new SpeakerShoot();
  }

  

}