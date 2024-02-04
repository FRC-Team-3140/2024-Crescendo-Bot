// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetArmToAngle;
import frc.robot.libs.XboxCotroller;
// // import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
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
public class RobotContainer {
  public static XboxCotroller controller = new XboxCotroller(0);
  public static AHRS gyro = new AHRS(Port.kMXP);
  public static SwerveDrive swerve = new SwerveDrive();
  // // private final Camera camera;
  private final Arm arm = Arm.getInstance();
  SendableChooser<Command> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // // camera = Camera.getInstance();
    autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    autoChooser.addOption("Auto2", new PathPlannerAuto("Auto2"));
    autoChooser.addOption("Auto3", new PathPlannerAuto("Auto3"));

    SmartDashboard.putData("Auto", autoChooser);

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
    new JoystickButton(controller, Button.kA.value).onTrue(new SetArmToAngle(8));
    new JoystickButton(controller, Button.kB.value).onTrue(new SetArmToAngle(10));
    new JoystickButton(controller, Button.kY.value).onTrue(new SetArmToAngle(94));
    new JoystickButton(controller, Button.kX.value).onTrue(new SetArmToAngle(50));
    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand((this::resetGyro)));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(()-> swerve.resetPose(new Pose2d())));
  }

  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *         //
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
