// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

public class turnToFaceApriltag extends Command {
  private SwerveDrive swerve;
  private Camera camera;
  private boolean complete = false;
  private int ID = -1;

  private double speed;
  private double degrees = 0;

  /** Creates a new turnToFaceApriltag. */
  public turnToFaceApriltag(double driveSpeed, SwerveDrive swerveDrive, Camera cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, cam);

    swerve = swerveDrive;
    camera = cam;

    speed = driveSpeed;
  }

  /** Creates a new turnToFaceApriltag. */
  public turnToFaceApriltag(double driveSpeed, int id, SwerveDrive swerveDrive, Camera cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cam);
    addRequirements(RobotContainer.swerve);
    swerve = swerveDrive;
    camera = cam;
    ID = id;

    speed = driveSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ID != -1) {
      degrees = camera.getDegToApriltag(ID);
    } else {
      degrees = camera.getDegToApriltag();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("!!!!!!!!!!!!!!!!!!!!\nTURNING\n!!!!!!!!!!!!!!!!!!!!");

    degrees = swerve.getPose().getRotation().getDegrees() + degrees;
 
    if (swerve.getPose().getRotation().getDegrees() > degrees) {
      swerve.drive(0, 0, -speed, false);
    } else if (swerve.getPose().getRotation().getDegrees() < degrees) {
      swerve.drive(0, 0, speed, false);
    } else {
      complete = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (complete) {
      return true;
    } else {
      return false;
    }
  }
}
