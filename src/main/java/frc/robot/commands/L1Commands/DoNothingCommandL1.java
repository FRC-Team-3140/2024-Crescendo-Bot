package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class DoNothingCommandL1 extends Command {
    SwerveDrive swerve = SwerveDrive.getInstance();
    public DoNothingCommandL1(){
        addRequirements(swerve);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
