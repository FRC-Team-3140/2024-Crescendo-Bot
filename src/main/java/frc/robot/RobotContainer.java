
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
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.OneNoteAuto;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootAmpL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
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


/* TODO: General notes...
*
* FIRST: TESTING is critical.  We need to test the robot in a variety of
* situations and make sure it is reliable.  Teleop needs to be bulletproof. We need to test the autonomous 
* modes on both sides. You need to find a way to test more often, test faster, and push the bot harder.
*
* SECOND: BROWNOUTS. I am paying paticular attention to current limits since I think we had 2-3 matches with
* brownouts.  Please do some serious testing and try to figure out which subsystems really 
* need the power and trim back on these limits where possible.  Time needs to be spent on this.
*
* THIRD: AUTOS. SMR demonstrated we need a complete set of autos that don't depend on path planner.  
* We don't have a lot of time to implement these so keep it simple, get it done and test.
*
* Don't make things too complicated.  The code is already hard to understand.  Try to make it 
* simpler where possible.  If you keep having to add patches and fix gitch after glitch, then
* you are probably doing something wrong and you need to simplify or find an alternative.
*  
* SwerveDrive: This is a very complicated subsystem.  If you want to keep path planner I think you need to totally cut
* out the camera.  Odometry needs to be simple. It is too much to have both.  I think you need to simplify the swerve drive and make it more reliable.
*
* See my comments in the Shooter class.  Keep the subsystem simple and reliable.  
* If you want to do something complicated put it in a Command.
* 
* If you integrate navigation with the camera it really needs to be in an isolated subsystem.  It should 
* have a seperate odometer and protections against bad data and crashes. It needs to be non critical to 
* the core operations of the bot.
*
* Clean out old, unneeded, or dead code.  It makes the code hard to read and understand. It is also 
* a source of bugs.
*
* "public static" is in general bad.  You are making global varibles.  Better to make a singleton and pass references to the objects that need them.
* 
* "public" members are also bad and indicate a lack of encapsulation. Better to make them private and control access with getter/setter methods.
* 
* COPILOT HINT: Network table updates can be put in try/catch blocks to avoid crashing the robot code if the network table is not available.
* 
* COPILOT HINT: Make the "catch" just print out a simple error message.  Don't create a new error with a complicated catch block.
*/


/* 
 * TODO: Apply Copilot's suggestions for improvements:
 * 
 * 1. Singleton Pattern: Implement the Singleton pattern for the RobotContainer class. 
 *    This ensures only one instance of RobotContainer exists, preventing issues with 
 *    multiple instances modifying the subsystems.
 *
 * 2. Access Modifiers: The subsystems and controllers are currently public static, 
 *    meaning they can be accessed and modified from anywhere. It would be better to 
 *    make these private and provide public getter methods to control access.
 *
 * 3. Remove Unused Code: Remove the commented out code to keep the codebase clean 
 *    and easy to read.
 *
 * 4. Spelling and Grammar: Correct the spelling and grammar in the comments. 
 *    For example, "commppands" should be "commands".
 */

/**
 * This class is primarily for declaring the structure of the robot in a Command-based paradigm. 
 * It should contain declarations for subsystems, commands, and trigger mappings. 
 * Most of the robot logic should not be handled in the {@link Robot} periodic methods, 
 * apart from the scheduler calls.
 */
public class RobotContainer implements Constants {
  // TODO: "public static" everywhere.  Better to make this a singleton and put more controls on who has access to these.
  public static XboxCotroller controller = new XboxCotroller(0);
  public static SwerveDrive swerve;
  public static Camera camera;
  public static Arm arm = Arm.getInstance();
  // private final Camera camera; // TODO Clean up commented out code
  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Command> autobuilder;
  public static Climber climber = Climber.getInstance();

  public static XboxCotroller controller2 = new XboxCotroller(1);
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
        new ParallelRaceGroup(new SpeakerShootDistanceL3(), new WaitCommand(3)));

    NamedCommands.registerCommand("SpeakerShoot3",
        new ParallelCommandGroup(new SetArmToAngleL1(18), new ShootSpeakerL1(10., 3)));

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


  // TODO: Do we have a single place where controls are defined?  How do we communicate to the driver what the controls are?
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
    BooleanSupplier rightTriggerC1 = () -> controller.getRightTriggerAxis() > .3; // TODO: Not used
    BooleanSupplier leftTriggerC1 = () -> controller.getLeftTriggerAxis() > .3;

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
    new JoystickButton(controller2, Button.kStart.value).whileTrue(new RepeatCommand(new SetArmToDistanceL1()));
    new JoystickButton(controller2, Button.kA.value).onTrue(new SetArmToAngleL1(16)).onTrue(new InstantCommand(()-> {if(DriverStation.getAlliance().get().equals(Alliance.Blue)){swerve.resetPose(new Pose2d(1.3,5.5, new Rotation2d()));}
    else{
      swerve.resetPose(new Pose2d(15.28,5.58,new Rotation2d()));
    }
  })); //Optimal angle for shooting from against the speaker.  
    new POVButton(controller2, 0).whileTrue(new RepeatCommand(new SetArmToDistanceWhileMovingL2()));
    //Intake/Shooter Controls     
    new JoystickButton(controller2, Button.kRightBumper.value).onTrue(new ShootAmpL1()).onFalse(new ShootSpeakerL1(0, 0));//.onFalse(new ShootSpeakerL1(0,0));
    new JoystickButton(controller2, Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new IntakeUntilNoteDetectedL1(), new SetArmToAngleL1(Arm.kSetpointMove)));
    BooleanSupplier rightTriggerC2 = () -> (controller2.getRightTriggerAxis() > 0.3);
    BooleanSupplier lefttTriggerC2 = () -> (controller2.getLeftTriggerAxis() > 0.3);
    new Trigger(lefttTriggerC2).onTrue(new ShootSpeakerOverrideL1(9.6,5)).onFalse(new ShootSpeakerL1(0, 0));//.onFalse(new ShootSpeakerL1(0, 0));
    BooleanSupplier upControllerLeftC2 = () -> (controller2.getLeftY() > 0.3);
    BooleanSupplier downControllerLeftC2 = () -> (controller2.getLeftY() < -0.3);
    BooleanSupplier upControllerRightC2 = () -> (controller2.getRightY() > 0.3);
    BooleanSupplier downControllerRightC2 = () -> (controller2.getRightY() < -0.3);
    
    new Trigger(upControllerLeftC2).onTrue(climber.increaseLeftHeight())
        .onFalse(new InstantCommand(climber::stopLeft));
    new Trigger(upControllerRightC2).onTrue(climber.increaseRightHeight())
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(downControllerRightC2).onTrue(new InstantCommand(climber::lowerRight))
        .onFalse(new InstantCommand(climber::stopRight));
    new Trigger(downControllerLeftC2).onTrue(new InstantCommand(climber::lowerLeft))
        .onFalse(new InstantCommand(climber::stopLeft));

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