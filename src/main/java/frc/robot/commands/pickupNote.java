// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

public class pickupNote extends Command implements Constants {
  /** Creates a new pickupNote. */
  private Intake intake = null;
  private SwerveDrive swerve = null;
  private Camera camera = null;

  public pickupNote(SwerveDrive swerve, Intake intake, Camera camera) {
    // TODO: sort command into respective difficulty levels if neccessary 
    this.swerve = swerve;
    this.intake = intake;
    this.camera = camera;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, intake, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SetArmToAngleL1(Arm.kSetpointIntakeDown).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(0, 0.25, camera.getNoteAngle(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.noteDetected();
  }
}
