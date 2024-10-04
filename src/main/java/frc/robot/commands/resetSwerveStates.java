// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class resetSwerveStates extends Command {
  private SwerveDrive swerveDrive = null;
  private boolean finish = false; 

  /** Creates a new resetSwerveStates. */
  public resetSwerveStates(SwerveDrive swerveDrive, boolean finish) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);

    this.swerveDrive = swerveDrive;
    this.finish = finish;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0)),
        new SwerveModuleState(0, new Rotation2d(0))
    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
