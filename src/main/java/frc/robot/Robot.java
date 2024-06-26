// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.SetClimberToTopL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.StopSpinningShooter;
import frc.robot.commands.L1Commands.ZeroClimbersL1;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void robotInit() {
    // DataLogManager.start();
    //DO NOT USE UNTIL COMPETITION

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  Tracer tracer = new Tracer();

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // System.out.println("pe" + photoElectric.get());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    // Put the arm in a safe position
    Arm.getInstance().disable();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Arm.getInstance().enable();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Ready the arm for movement.
    Arm.getInstance().enable();
  }

  // IntakeAndShooter test = IntakeAndShooter.getInstance();

  @Override
  public void testInit() {
    // test.intake(.6);

    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();

    // // Ready the arm for movement.
    Arm.getInstance().enable();
    new SetArmToAngleL1(NetworkTableInstance.getDefault().getTable("Double").getEntry("Test").getDouble(2)).schedule();

    // turnToFaceApriltag test - TK

    // new turnToFaceApriltag(6, SwerveDrive.getInstance(),
    // Camera.getInstance()).schedule();

    // pathfinToApriltag test - TK

    // double dist = Camera.getInstance().getAprilTagDist();
    // double botRot =
    // SwerveDrive.getInstance().getPose().getRotation().getRadians();
    // double aprilTagRot = Math.toRadians(Camera.getInstance().getDegToApriltag());
    Arm.getInstance().enable();
    // new pathfindToPose(
    // new Pose2d(-(dist * Math.cos(botRot + aprilTagRot)) +
    // SwerveDrive.getInstance().getPose().getX(), -(dist * Math.sin(botRot +
    // aprilTagRot)) + SwerveDrive.getInstance().getPose().getY(), new
    // Rotation2d(botRot + aprilTagRot)),
    // Camera.getInstance(), SwerveDrive.getInstance()).schedule();

    // Camera.getInstance().pathfindToAprilTag().schedule();

    // Climber climber = Climber.getInstance();

    // Test Command: Run before each round
    new SequentialCommandGroup(
    new SetArmToAngleL1(80),
    new ShootSpeakerL1(6.5,5).withTimeout(3),
    new ParallelRaceGroup(new InstantCommand(() -> {
    Intake.getInstance().setIntakeVoltage(0);
    }), 
    new StopSpinningShooter(), new WaitCommand(1)),
    new SetArmToAngleL1(10),
    new ZeroClimbersL1(),
    new WaitCommand(3),
    new SetClimberToTopL1(),
    new WaitCommand(3),
    new ZeroClimbersL1(),
    AutoBuilder.buildAuto("Straight Line"),
    AutoBuilder.buildAuto("Turn")).schedule();

    // pickupNote test with pathplanner
    // new SequentialCommandGroup(
    //     AutoBuilder.buildAuto("Straight Line").withTimeout(2), new PrintCommand("Straight Line has Ended"),
    //     new SetArmToAngleL1(20), 
    //     new pickupNote(false, RobotContainer.swerve,
    //         RobotContainer.camera), AutoBuilder.buildAuto("Opposite of Straight Line"),new PrintCommand("The Command Has Ended")
    //     )
    //     .schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // new SequentialCommandGroup(
    // new ParallelCommandGroup(new RepeatCommand(()-> SwerveDrive.)))

    // ).schedule();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
