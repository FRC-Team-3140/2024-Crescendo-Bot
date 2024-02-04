
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.DefaultShoot;
import frc.robot.commands.IntakeUntilNoteDetected;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;


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
 // public static XboxCotroller controller = new XboxCotroller(0);
 // public static AHRS gyro = new AHRS(Port.kMXP);
 // public static SwerveDrive swerve = new SwerveDrive();
 // private final Camera camera;
 // SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static XboxController xbox = Robot.xbox;
  public static IntakeShooter intakeShooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    intakeShooter = IntakeShooter.getInstance();

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
    new JoystickButton(xbox, Button.kX.value).onTrue(new SpeakerShoot()).onFalse(new InstantCommand(()-> {intakeShooter.setShooterVoltage(0);}));
    new JoystickButton(xbox, Button.kY.value).onTrue(new AmpShoot()).onFalse(new InstantCommand(()-> {intakeShooter.setShooterVoltage(0);}));
    new JoystickButton(xbox, Button.kB.value).onTrue(new DefaultShoot(0)).onFalse(new InstantCommand(()-> {intakeShooter.setShooterVoltage(0);}));


    new JoystickButton(xbox, Button.kA.value).onTrue(new IntakeUntilNoteDetected());
  } 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

 */ 

}