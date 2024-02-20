package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class StopLeft extends Command {
    Climber climber = Climber.getInstance();
    StopLeft(){
        addRequirements(climber);
    }
    @Override
    public void initialize() {
        climber.stopLeft();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
