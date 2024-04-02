// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

public class turnToFaceApriltag extends Command {
  private SwerveDrive swerve;
  // private Camera camera;
  // private int ID = -1;

  // private double degrees = 0;

  // private turnToTurnPIDSetPoint command;

  /** Creates a new turnToFaceApriltag. */
  public turnToFaceApriltag(SwerveDrive swerveDrive, Camera cam) {
    // TODO: sort command into respective difficulty levels if neccessary 
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(swerveDrive, cam);

    // swerve = swerveDrive;
    // camera = cam;
  }

  /** Creates a new turnToFaceApriltag. */
  public turnToFaceApriltag(int id, SwerveDrive swerveDrive, Camera cam) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(swerveDrive, cam);
    // swerve = swerveDrive;
    // camera = cam;
    // ID = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.turnToAprilTag(7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return Math.abs(Camera.getInstance().getApriltagYaw(7) - 5) < 0;
  }
}
