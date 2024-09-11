package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class FollowPathplannerPath extends Command {
  private String pathName;
  private SwerveDrive swerveDrive;
  private Command path;
  private boolean scheduled = false;

  public FollowPathplannerPath(String pathName, SwerveDrive swerveDrive) {
    addRequirements(swerveDrive);
    this.pathName = pathName;
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void initialize() {
    // Initialization logic if needed
  }

  @Override
  public void execute() {
    if (!scheduled && pathName != null) {
      if (path != null) {
        new FollowPathWithEndCommand(swerveDrive, PathPlanner.load).schedule();
        scheduled = true;
      } else {
        System.out.println("Received null command...");
      }
    } else if (pathName == null) {
      System.out.println("Cannot follow path because path not specified!");
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return scheduled && (path == null || !CommandScheduler.getInstance().isScheduled(path));
  }

  private class FollowPathWithEndCommand extends SequentialCommandGroup {
    public FollowPathWithEndCommand(SwerveDrive swerveDrive, PathPlannerTrajectory trajectory) {
      addCommands(
          new PPSwerveControllerCommand(
              trajectory,
              swerveDrive::getPose,
              swerveDrive.kinematics,
              new PIDController(1.0, 0.0, 0.0),
              new PIDController(1.0, 0.0, 0.0),
              new PIDController(1.0, 0.0, 0.0),
              swerveDrive::setModuleStates,
              swerveDrive),
          new InstantCommand(() -> swerveDrive.setModuleStates(new SwerveModuleState[] {
              new SwerveModuleState(0, new Rotation2d(0)),
              new SwerveModuleState(0, new Rotation2d(0)),
              new SwerveModuleState(0, new Rotation2d(0)),
              new SwerveModuleState(0, new Rotation2d(0))
          })));
    }
  }

}
