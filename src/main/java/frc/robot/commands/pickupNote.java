// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.L1Commands.IntakeUntilNoteDetectedL1;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Camera.ShapeData;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class has the provides the option to pass in a drive speed and
 * exploration timeout duration.
 * - Default drive speed is 3
 * - Set in driveSpeed variable
 *
 * - Default timeout is 2 SECONDS!
 * - Set in exploreTimeout
 * This timeout is only used when no notes are in frame!
 * 
 * - Global Timeout: Default 4 SECONDS!
 * - Ends command after set duration
 * 
 * @param withController Wether or not to drive with strafing controller inputs
 * @param swerve         The SwerveDrive subsystem
 * @param camera         The Camera subsystem
 * @param exploreTimeout The amount of time to drive without a note in frame
 * @param driveSpeed     The constant speed to drive at
 */

public class pickupNote extends SequentialCommandGroup {
  /** Creates a new pickupNote. */
  public pickupNote(Boolean withController, SwerveDrive swerve, Camera camera) {
    // TODO: sort command into respective difficulty levels if neccessary
    // super(new InstantCommand(() -> {
    // // pickupNote.swerve = swerve;
    // pickupNote.swerve = SwerveDrive.getInstance();
    // // pickupNote.camera = camera;
    // pickupNote.camera = Camera.getInstance();

    // pickupNote.withController = withController;

    // System.out.println("With controller: " +
    // pickupNote.withController.toString());
    // }), !withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
    // SequentialCommandGroup(),
    // new ParallelDeadlineGroup(new PickUpNoteCommand(),
    // !withController ? new IntakeUntilNoteDetectedL1() : new
    // SequentialCommandGroup()));
    // pickupNote.swerve = swerve;
    // pickupNote.swerve = SwerveDrive.getInstance();
    // // pickupNote.camera = camera;
    // pickupNote.camera = Camera.getInstance();

    // pickupNote.withController = withController;

    // addCommands(new InstantCommand(() -> {
    //   pickupNote.swerve = swerve;
    //   // pickupNote.swerve = SwerveDrive.getInstance();
    //   pickupNote.camera = camera;
    //   // pickupNote.camera = Camera.getInstance();

    //   pickupNote.withController = withController;

    //   swerve.drive(0, 0, 0, false);
    // }), new PrintCommand("With Controller: " + pickupNote.withController.toString()),
    //     !withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new SequentialCommandGroup(),
    //     new ParallelDeadlineGroup(new PickUpNoteCommand(SwerveDrive.getInstance(), Camera.getInstance()),
    //         !withController ? new IntakeUntilNoteDetectedL1() : new SequentialCommandGroup())); 
    
    // TODO: Fix null passed in randomly for swerve error 
    if(!withController) {
      addCommands(new SetArmToAngleL1(Arm.kSetpointIntakeDown), new ParallelCommandGroup(new IntakeUntilNoteDetectedL1(), new PickUpNoteCommandWithoutController(SwerveDrive.getInstance(), Camera.getInstance())));  
    } else {
      addCommands(new PickUpNoteCommandWithController(SwerveDrive.getInstance(), Camera.getInstance()));
    }
  }

  // public pickupNote(Boolean withController, SwerveDrive swerve, boolean
  // returnToStart, Camera camera) {
  // // TODO: sort command into respective difficulty levels if neccessary
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.returnToStart = returnToStart;
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive
  // swerve, Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;
  // pickupNote.exploreTimeout = exploreTimeout;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, double exploreTimeout, boolean
  // returnToStart, SwerveDrive swerve,
  // Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.returnToStart = returnToStart;
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;
  // pickupNote.exploreTimeout = exploreTimeout;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, SwerveDrive swerve, double
  // driveSpeed, Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.driveSpeed = Math.max(Math.min(driveSpeed,
  // Constants.maxChassisSpeed), pickupNote.minDriveSpeed);
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, SwerveDrive swerve, boolean
  // returnToStart, double driveSpeed,
  // Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.returnToStart = returnToStart;
  // pickupNote.driveSpeed = Math.max(Math.min(driveSpeed,
  // Constants.maxChassisSpeed), pickupNote.minDriveSpeed);
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, double exploreTimeout, SwerveDrive
  // swerve, double driveSpeed,
  // Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.driveSpeed = Math.max(Math.min(driveSpeed,
  // Constants.maxChassisSpeed), pickupNote.minDriveSpeed);
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;
  // pickupNote.exploreTimeout = exploreTimeout;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  // public pickupNote(Boolean withController, double exploreTimeout, boolean
  // returnToStart, SwerveDrive swerve,
  // double driveSpeed,
  // Camera camera) {
  // super(!withController ? new SetArmToAngleL1(Arm.kSetpointIntakeDown) : new
  // SequentialCommandGroup(),
  // new ParallelDeadlineGroup(new PickUpNoteCommand(),
  // !withController ? new IntakeUntilNoteDetectedL1() : new
  // SequentialCommandGroup()));

  // pickupNote.swerve = swerve;
  // pickupNote.returnToStart = returnToStart;
  // pickupNote.driveSpeed = Math.max(Math.min(driveSpeed,
  // Constants.maxChassisSpeed), pickupNote.minDriveSpeed);
  // pickupNote.camera = camera;

  // pickupNote.withController = withController;
  // pickupNote.exploreTimeout = exploreTimeout;

  // // Use addRequirements() here to declare subsystem dependencies.
  // addRequirements(swerve);
  // }

  private static class PickUpNoteCommandWithController extends Command {
    private boolean run = true;

    private SwerveDrive swerve = null;
    private Camera camera = null;

    private double driveSpeed = 2;
    private double minDriveSpeed = 0.3;

    private boolean returnToStart = false;
    private Pose2d startingPose = null;

    private IntakeUntilNoteDetectedL1 intakeCommand = new IntakeUntilNoteDetectedL1();

    private PIDController turnController = new PIDController(0.025, 0.01, 0.0045);

    private double deadzone = 0.25;

    private double globalTimeout = 4;
    private double exploreTimeout = 2;

    private ShapeData shapeData;

    private double ang = 0;

    private double driveAng;

    public PickUpNoteCommandWithController(SwerveDrive swerve, Camera camera) {
      this.swerve = swerve; 

      this.camera = camera; 

      addRequirements(swerve, camera);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      run = true;

      startingPose = swerve.getPose();
      
      turnController.setSetpoint(swerve.getPose().getRotation().getDegrees());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      try {
        shapeData = camera.getShapeData();

        if (shapeData != null) {
          if (Math.abs(shapeData.angle) > deadzone) {
            ang = shapeData.angle;
          }

          turnController.setSetpoint(swerve.getPose().getRotation().getDegrees() + ang);
        } else {
          System.out.println("Not valid angle returned!");
        }

        driveAng = -turnController.calculate(swerve.getPose().getRotation().getDegrees());

        if (RobotContainer.controller.getRightTriggerAxis() >= .3) {
          if (!intakeCommand.isScheduled()) {
            intakeCommand.schedule();
          }

          swerve.drive(driveSpeed, 0, shapeData != null ? (Math.pow((1 - (shapeData.area / 100)), 2) * driveAng) : 0,
              false);
        } else {
          if (intakeCommand.isScheduled()) {
            intakeCommand.cancel();
          }
          swerve.drive(-(RobotContainer.controller.getLeftY() * Constants.maxChassisSpeed),
              -(RobotContainer.controller.getLeftX() * Constants.maxChassisSpeed),
              shapeData != null ? (Math.pow((1 - (shapeData.area / 100)), 2) * driveAng)
                  : -(RobotContainer.controller.getRightX() * Constants.maxChassisTurnSpeed),
              true);
        }
      } catch (Error e) {
        System.out.println("An error has occured in pickupNote: \n" + e);
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (returnToStart && startingPose != null) {
        AutoBuilder.pathfindToPose(startingPose, new PathConstraints(Constants.maxChassisSpeed, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720))).schedule();
      };
      System.out.println("Stopping Intake");
      Intake.getInstance().setIntakeVoltage(0);
      swerve.drive(0, 0, 0, false);
      System.out.println("Ended!");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (run) {
        return Intake.getInstance().noteDetected();
      } else {
        return true;
      }
    }
  }

  private static class PickUpNoteCommandWithoutController extends Command {
    private boolean run = true;

    private Timer timeout = new Timer();
    private Timer globalTimer = new Timer();

    private SwerveDrive swerve = null;
    private Camera camera = null;

    private double driveSpeed = 2;
    private double minDriveSpeed = 0.3;

    private boolean returnToStart = false;
    private Pose2d startingPose = null;

    // Run with SwerveDrive Controller
    private Boolean withController = false;

    private PIDController turnController = new PIDController(0.025, 0.01, 0.0045);

    private double deadzone = 0.25;

    private double globalTimeout = 4;
    private double exploreTimeout = 2;

    private ShapeData shapeData;

    private double ang = 0;

    private double driveAng;

    public PickUpNoteCommandWithoutController(SwerveDrive swerve, Camera camera) {
      this.swerve = swerve; 

      this.camera = camera; 

      addRequirements(swerve, camera);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      run = true;

      startingPose = swerve.getPose();
      if (!withController) {
        globalTimer.start();
      }
      
      turnController.setSetpoint(swerve.getPose().getRotation().getDegrees());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      try {
        shapeData = camera.getShapeData();

        if (shapeData != null) {
          if (Math.abs(shapeData.angle) > deadzone) {
            ang = shapeData.angle;
          }

          turnController.setSetpoint(swerve.getPose().getRotation().getDegrees() + ang);
        } else {
          System.out.println("Not valid angle returned!");
        }

        driveAng = -turnController.calculate(swerve.getPose().getRotation().getDegrees());

        if (!camera.getShapeDetected()) {
          timeout.start();
        } else {
          timeout.stop();
          timeout.reset();
        }

        if (timeout.hasElapsed(exploreTimeout)) {
          run = false;
        } else {
          swerve.drive(driveSpeed, 0, shapeData != null ? (Math.pow((1 - (shapeData.area / 100)), 2) * driveAng) : 0,
              false);
        }
      } catch (Error e) {
        System.out.println("An error has occured in pickupNote: \n" + e);
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (returnToStart && startingPose != null) {
        AutoBuilder.pathfindToPose(startingPose, new PathConstraints(Constants.maxChassisSpeed, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720))).schedule();
      }
      timeout.stop();
      timeout.reset();
      globalTimer.stop();
      globalTimer.reset();
      System.out.println("Stopping Intake");
      Intake.getInstance().setIntakeVoltage(0);
      swerve.drive(0, 0, 0, false);
      System.out.println("Ended!");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (!withController && globalTimer.hasElapsed(globalTimeout)) {
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
