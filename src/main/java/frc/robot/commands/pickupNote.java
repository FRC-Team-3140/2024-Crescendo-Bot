// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class pickupNote extends SequentialCommandGroup {
  /** Creates a new pickupNote. */
  private static boolean run = true;

  private static Timer timeout = new Timer();
  private static Timer globalTimer = new Timer();

  private static SwerveDrive swerve = null;
  private static Camera camera = null;

  private static double driveSpeed = 3;

  // Run with SwerveDrive Controller
  private static Boolean withController = false;

  private static PIDController turnController = new PIDController(0.025, 0, 0.0025);

  private static double deadzone = 5;

  private static double globalTimeout = 4;
  private static double exploreTimeout = 2;

  /***********************************************************************
   * *
   * This class has the provides the option to pass in a drive speed and *
   * exploration timeout duration. *
   * - Default drive speed is 3 *
   * - Set in driveSpeed variable *
   * *
   * - Default timeout is 2 SECONDS! *
   * - Set in exploreTimeout *
   * * This timeout is only used when no notes are in frame! *
   * *
   ***********************************************************************/

  public pickupNote(Boolean withController, SwerveDrive swerve, Camera camera) {
    // TODO: sort command into respective difficulty levels if neccessary
    super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new SequentialCommandGroup(),
        new ParallelRaceGroup(!withController ? new IntakeUntilNoteDetectedL1() : new SequentialCommandGroup(),
            new PickUpNoteCommand()));

    pickupNote.swerve = swerve;
    pickupNote.camera = camera;

    pickupNote.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive swerve, Camera camera) {
    super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new SequentialCommandGroup(),
        new ParallelRaceGroup(!withController ? new IntakeUntilNoteDetectedL1() : new SequentialCommandGroup(),
            new PickUpNoteCommand()));

    pickupNote.swerve = swerve;
    pickupNote.camera = camera;

    pickupNote.withController = withController;
    pickupNote.exploreTimeout = exploreTimeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public pickupNote(Boolean withController, SwerveDrive swerve, double driveSpeed, Camera camera) {
    super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new SequentialCommandGroup(),
        new ParallelRaceGroup(!withController ? new IntakeUntilNoteDetectedL1() : new SequentialCommandGroup(),
            new PickUpNoteCommand()));

    pickupNote.swerve = swerve;
    pickupNote.driveSpeed = driveSpeed;
    pickupNote.camera = camera;

    pickupNote.withController = withController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive swerve, double driveSpeed,
      Camera camera) {
    super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new SequentialCommandGroup(),
        new ParallelRaceGroup(!withController ? new IntakeUntilNoteDetectedL1() : new SequentialCommandGroup(),
            new PickUpNoteCommand()));

    pickupNote.swerve = swerve;
    pickupNote.driveSpeed = driveSpeed;
    pickupNote.camera = camera;

    pickupNote.withController = withController;
    pickupNote.exploreTimeout = exploreTimeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  private static class PickUpNoteCommand extends Command {
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      run = true;
      timeout.stop();
      timeout.reset();
      globalTimer.stop();
      globalTimer.reset();
      globalTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      try {
        double ang = camera.getNoteAngle();

        double driveAng;

        if (ang != 999) {
          if (Math.abs(ang) < deadzone) {
            ang = 0;
          }

          turnController.setSetpoint(swerve.getPose().getRotation().getDegrees() + ang);
        } else {
          System.out.println("Not valid angle returned!");
        }

        driveAng = -turnController.calculate(swerve.getPose().getRotation().getDegrees());

        if (withController) {
          swerve.drive(-(RobotContainer.driver_controller.getLeftY() * Constants.maxChassisSpeed),
              -(RobotContainer.driver_controller.getLeftX() * Constants.maxChassisSpeed),
              Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng,
              true);
        } else {
          if (!camera.getNoteDetected()) {
            timeout.start();
          } else {
            timeout.stop();
            timeout.reset();
          }

          if (timeout.hasElapsed(exploreTimeout)) {
            run = false;
          } else {
            swerve.drive(driveSpeed, 0, Math.pow((1 - (camera.getNoteArea() / 100)), 2) * driveAng, false);
          }
        }
      } catch (Error e) {
        System.out.println("An error has occured in pickupNote: \n" + e);
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      timeout.stop();
      timeout.reset();
      globalTimer.stop();
      globalTimer.reset();
      Intake.getInstance().setIntakeVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (globalTimer.hasElapsed(globalTimeout)) {
        return true;
      }

      if (run) {
        return Intake.getInstance().noteDetected();
      } else {
        return true;
      }
    }
  }
}
