// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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

  private double driveSpeed = 0.25;

  // Run with SwerveDrive Controller
  private Boolean withController = false;

  private PIDController turnController = new PIDController(0.025, 0, 0.0025);

  private double deadzone = 5;

  /********************************************************************
   * *
   * This class has the provides the option to pass in a drive speed. *
   * - Default is 0.25 *
   * - Set in driveSpeed variable *
   * *
   ********************************************************************/

  public pickupNote(Boolean withController, SwerveDrive swerve, Intake intake, Camera camera) {
    // TODO: sort command into respective difficulty levels if neccessary
    this.swerve = swerve;
    this.camera = camera;

    this.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  public pickupNote(Boolean withController, SwerveDrive swerve, double driveSpeed, Intake intake, Camera camera) {
    this.swerve = swerve;
    this.driveSpeed = driveSpeed;
    this.camera = camera;

    this.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SetArmToAngleL1(Arm.kSetpointIntakeDown).schedule();
    new IntakeUntilNoteDetectedL1().schedule();
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

    if (withController) {
      swerve.drive(-(RobotContainer.controller.getLeftX() * Constants.maxChassisSpeed),
          -(RobotContainer.controller.getLeftY() * Constants.maxChassisSpeed),
          Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng,
          true);
    } else {
      // TODO: Fix angle so it uses PID controller - TK
      swerve.drive(0, driveSpeed, Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.controller.getLeftBumper()) {
      return true;
    }
    return Intake.getInstance().noteDetected();
  }
}
