package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class FollowPathplannerPath extends Command {
  private String pathName = null;
  private SwerveDrive swerveDrive = null;

  private Command path = null;
  private SequentialCommandGroup finalCommand = null;
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
    try {
      if (!scheduled && (pathName != null || pathName.equals(""))) {
        path = AutoBuilder.buildAuto(pathName);
        if (path != null) {
          finalCommand = new SequentialCommandGroup(
              path,
              new InstantCommand(() -> swerveDrive.setModuleStates(new SwerveModuleState[] {
                  new SwerveModuleState(0, new Rotation2d(0)),
                  new SwerveModuleState(0, new Rotation2d(0)),
                  new SwerveModuleState(0, new Rotation2d(0)),
                  new SwerveModuleState(0, new Rotation2d(0))
              })));
          finalCommand.schedule();
          scheduled = true;
        } else {
          System.out.println("Received null command...");
        }
      } else if ((pathName != null || pathName.equals(""))) {
        System.out.println("Cannot follow path because path not specified!");
      }
    } catch (Exception e) {
      System.out.println("An error occrued while attempting to follow path:\n" + e);
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    // return scheduled && (path == null ||
    // !CommandScheduler.getInstance().isScheduled(path));
    return finalCommand.isFinished();
  }
}
