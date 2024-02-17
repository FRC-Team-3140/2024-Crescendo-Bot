// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkBase.IdleMode;
// import frc.robot.libs.XboxCotroller;
// import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot implements Constants{
  // private Command m_autonomousCommand;
  private RobotContainer m_robotContainer; 
  //  public static CANSparkMax left = new CANSparkMax(9, MotorType.kBrushless);
  //  public static CANSparkMax right = new CANSparkMax(8, MotorType.kBrushless);
public Climber climber = new Climber();
// public Climber right = new Climber();
   NetworkTableInstance inst = NetworkTableInstance.getDefault();

  
    NetworkTable motorInfo = inst.getTable("motorinfo");
   
   public XboxController x = new XboxController(0);
  // private static XboxCotroller m_controller = RobotContainer.controller;
  // private static SwerveDrive swerve = RobotContainer.swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    // swerve.updateOdometry();
  }
  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    // driveWithJoystick(true);

    //trmporary code
    if(x.getAButton() || x.getBButton() || x.getXButton() || x.getYButton()){
    
      if(x.getAButton()){
        climber.lowerLeft();
      }

      if(x.getBButton()){
        climber.lowerRight();
      }

      if(x.getXButton()){
        climber.raiseLeft();
      }

      if(x.getYButton()){
        climber.raiseRight();
      }
    }else{
      climber.stopLeft();
      climber.stopRight();
    }
    


    // motorInfo.getEntry("leftvoltage").setDouble(left.getBusVoltage()*left.getAppliedOutput());
    // motorInfo.getEntry("leftcurrent").setDouble(left.getOutputCurrent());
    // double leftvoltage = motorInfo.getEntry("leftvoltage").getDouble(0);
    // double leftcurrent = motorInfo.getEntry("leftcurrent").getDouble(0);
    // motorInfo.getEntry("leftpower").setDouble(leftcurrent*leftvoltage);

    // motorInfo.getEntry("rightvoltage").setDouble(right.getBusVoltage()*right.getAppliedOutput());
    // motorInfo.getEntry("rightcurrent").setDouble(right.getOutputCurrent());
    // double rightvoltage = motorInfo.getEntry("rightvoltage").getDouble(0);
    // double rightcurrent = motorInfo.getEntry("rightcurrent").getDouble(0);
    // motorInfo.getEntry("rightpower").setDouble(rightcurrent*rightvoltage);
    
    
  }

  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    
    motorInfo.getEntry("leftvoltage").setDefaultValue(0);
    motorInfo.getEntry("leftcurrent").setDefaultValue(0);
    motorInfo.getEntry("leftpower").setDefaultValue(0);

    motorInfo.getEntry("rightvoltage").setDefaultValue(0);
    motorInfo.getEntry("rightcurrent").setDefaultValue(0);
    motorInfo.getEntry("rightpower").setDefaultValue(0);
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */

  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
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



  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // final var xSpeed = -m_controller.getLeftY() * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    // final var ySpeed = m_controller.getLeftX() * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot = -m_controller.getRightX() * maxChassisTurnSpeed;

    // swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
