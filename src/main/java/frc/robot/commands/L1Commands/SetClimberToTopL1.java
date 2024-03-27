package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class SetClimberToTopL1 extends Command {
    Climber climber = Climber.getInstance();
    public SetClimberToTopL1(){
        addRequirements(climber);
    }
    @Override
    public void initialize() {
        new ParallelCommandGroup(climber.increaseLeftHeight(), climber.increaseRightHeight()).schedule();

    }
    @Override
    public void end(boolean interrupted) {
        climber.stopBoth();
    }
    @Override
    public boolean isFinished() {
        return climber.bothReachedTop();
    }
}