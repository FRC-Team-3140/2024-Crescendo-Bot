package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimbersL1 extends Command {
    Climber climber = Climber.getInstance();
    double deadband = .001;
    public ZeroClimbersL1(){
        addRequirements(climber);
    }
    @Override
    public void initialize() {
        climber.lowerBoth();
    }
    @Override
    public boolean isFinished() {
        return climber.bothLimitSwitchesPressed();
    }
}