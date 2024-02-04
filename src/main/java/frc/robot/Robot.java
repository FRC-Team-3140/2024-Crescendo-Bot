// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.RobotContainer;


public class Robot extends TimedRobot implements Constants{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private IntakeShooter intakeShooter = IntakeShooter.getInstance();

  public static XboxController xbox = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  @Override
  public void autonomousPeriodic() {
   
  }

  @Override
  public void teleopPeriodic() {
    //intake speed: 0.5
    // if(xbox.getRightBumper()){
    //   intakeShooter.setIntakeVoltage(intakeVoltage);
    // }else {
    //   intakeShooter.setIntakeVoltage(0);
    // }

    if(xbox.getLeftBumper()){
      intakeShooter.setShooterVoltage(10);
    }else{
      intakeShooter.setShooterVoltage(0);
    }

 // shooter speed: 0.6
    // if(xbox.getLeftTriggerAxis()>0){
    //   intake.setShooterVoltage(xbox.getLeftTriggerAxis());
    //  System.out.println(xbox.getLeftTriggerAxis());
    // }else {
    //   intake.setShooterVoltage(0);;
    // }
  }

  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  }
  // IntakeAndShooter test = IntakeAndShooter.getInstance();
  @Override
  public void testInit() {
    // test.intake(.6);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}
