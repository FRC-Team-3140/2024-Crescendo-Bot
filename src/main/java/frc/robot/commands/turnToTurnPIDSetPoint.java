// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class turnToTurnPIDSetPoint extends Command implements Constants{
  private SwerveDrive swerve;
  private double setPoint;
  private double currentBotAngle;

  private final double deadband = 5;

  /** Creates a new turnToTurnPIDSetPoint. */
  public turnToTurnPIDSetPoint(SwerveDrive swerveDrive, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    swerve = swerveDrive;
    this.setPoint = setPoint;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.turnPID.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentBotAngle = swerve.getPose().getRotation().getDegrees();

    swerve.drive(-maxSpeed * RobotContainer.controller.getLeftY(), -maxSpeed * RobotContainer.controller.getLeftX(), swerve.turnPID.calculate(-currentBotAngle), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currentBotAngle - setPoint) < deadband) {
      return true;
    } else {
      return false;
    }
  }
}
