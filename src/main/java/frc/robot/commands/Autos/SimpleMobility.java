// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class SimpleMobility extends Command {
  private SwerveDrive swerveDrive = null;
  
  // private Command path = null;
  // private boolean scheduled = false;

  // private double forwardDistM = 0.0;
  private long startTime = System.currentTimeMillis(); 

  /** Creates a new SimpleMobility. */
  public SimpleMobility(SwerveDrive swerveDrive, double forwardDistM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);

    this.swerveDrive = swerveDrive;
    // this.forwardDistM = forwardDistM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pose2d currentPose2d = swerveDrive.getPose();

    // System.out.println(currentPose2d.getX() + " " + currentPose2d.getY() + " " + currentPose2d.getRotation().getDegrees());

    // Pose2d goal = new Pose2d(new Translation2d((currentPose2d.getX() + forwardDistM), currentPose2d.getY()),
    //     currentPose2d.getRotation());

    // System.out.println(goal.getX() + " " + goal.getY() + " " + goal.getRotation().getDegrees());

    // path = AutoBuilder.pathfindToPose(goal, new PathConstraints(Constants.maxChassisSpeed, Constants.maxAcceleration, Units.degreesToRadians(Constants.maxChassisTurnSpeed), Units.degreesToRadians(Constants.maxChassisTurnSpeed)));

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running");
    if (System.currentTimeMillis() <= startTime + 1000) {
      swerveDrive.drive(4, 0, 0, false);
    }
    // if (!scheduled) {
    //   path.schedule();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return path.isFinished();
    return System.currentTimeMillis() <= startTime + 1000;
  }
}
