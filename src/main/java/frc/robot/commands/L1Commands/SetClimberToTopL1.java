package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * A command that sets the climber to the top position for Level 1.
 */
public class SetClimberToTopL1 extends Command {
    Climber climber = Climber.getInstance();

    /**
     * Constructs a new SetClimberToTopL1 command.
     * Adds the climber as a requirement.
     */
    public SetClimberToTopL1(){
        addRequirements(climber);
    }

    /**
     * Initializes the command.
     * Schedules a parallel command group to increase the height of both sides of the climber.
     */
    @Override
    public void initialize() {
        new ParallelCommandGroup(climber.increaseLeftHeight(), climber.increaseRightHeight()).schedule();
    }

    /**
     * Ends the command.
     * Stops both sides of the climber.
     * @param interrupted true if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        climber.stopBoth();
    }

    /**
     * Checks if the command is finished.
     * @return true if both sides of the climber have reached the top position, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return climber.bothReachedTop();
    }
}