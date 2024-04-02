// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class pickupNote extends Command {
  /** Creates a new pickupNote. */
  private SwerveDrive swerve = null;
  private Camera camera = null;

  private double driveSpeed = 3;

  // Run with SwerveDrive Controller
  private Boolean withController = false;

  private PIDController turnController = new PIDController(0.025, 0, 0.0025);

  private double deadzone = 5;

  private double exploreTimeout = 2;

  /***********************************************************************
   *                                                                     *
   * This class has the provides the option to pass in a drive speed and *
   * exploration timeout duration.                                       *
   * - Default drive speed is 3                                          *
   * - Set in driveSpeed variable                                        *
   *                                                                     *
   * - Default timeout is 2 SECONDS!                                     *
   * - Set in exploreTimeout                                             *
   *    * This timeout is only used when no notes are in frame!          *
   *                                                                     *
   ***********************************************************************/

  public pickupNote(Boolean withController, SwerveDrive swerve, Camera camera) {
    // TODO: sort command into respective difficulty levels if neccessary
    this.swerve = swerve;
    this.camera = camera;

    this.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive swerve, Camera camera) {
    this.swerve = swerve;
    this.camera = camera;

    this.withController = withController;
    this.exploreTimeout = exploreTimeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  public pickupNote(Boolean withController, SwerveDrive swerve, double driveSpeed, Camera camera) {
    this.swerve = swerve;
    this.driveSpeed = driveSpeed;
    this.camera = camera;

    this.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive swerve, double driveSpeed,
      Camera camera) {
    this.swerve = swerve;
    this.driveSpeed = driveSpeed;
    this.camera = camera;

    this.withController = withController;
    this.exploreTimeout = exploreTimeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!withController) {
      new SetArmToAngleL1(Arm.kSetpointIntakeDown).schedule();
      new IntakeUntilNoteDetectedL1().schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ang = camera.getNoteAngle();

    turnController.setSetpoint(swerve.getPose().getRotation().getDegrees() + ang);

    double driveAng = -turnController.calculate(swerve.getPose().getRotation().getDegrees());

    if (Math.abs(ang) < deadzone) {
      driveAng = 0;
    }

    // double turnSpeed = 0.025 * camera.getNoteAngle();
    // turnSpeed = Math.min(Math.max(turnSpeed, -0.3 * Constants.maxChassisSpeed),
    // 0.3 * Constants.maxChassisSpeed);

    if (withController) {
      swerve.drive(-(RobotContainer.controller.getLeftY() * Constants.maxChassisSpeed),
          -(RobotContainer.controller.getLeftX() * Constants.maxChassisSpeed),
          Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng,
          false);
    } else {
      Timer timeout = new Timer();

      if (!camera.getNoteDetected()) {
        timeout.start();
      } else {
        timeout.stop();
        timeout.reset();
      }

      if (!timeout.hasElapsed(exploreTimeout)) {
        swerve.drive(driveSpeed, 0, Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng, false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Intake.getInstance().noteDetected();
  }
}
