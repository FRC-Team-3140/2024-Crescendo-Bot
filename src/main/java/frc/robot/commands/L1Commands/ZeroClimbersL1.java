package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * A command that lowers both climbers until their limit switches are pressed, effectively zeroing them.
 */
public class ZeroClimbersL1 extends Command {
    Climber climber = Climber.getInstance();
    double deadband = .001;

    /**
     * Constructs a new ZeroClimbersL1 command.
     * Adds the climber subsystem as a requirement.
     */
    public ZeroClimbersL1(){
        addRequirements(climber);
    }

    /**
     * Initializes the command by lowering both climbers.
     */
    @Override
    public void initialize() {
        climber.lowerBoth();
    }

    /**
     * Checks if both climbers' limit switches are pressed.
     * @return true if both limit switches are pressed, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return climber.bothLimitSwitchesPressed();
    }
}