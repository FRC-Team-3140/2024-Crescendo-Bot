package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopSpinningShooter extends Command{
    public StopSpinningShooter(){
        addRequirements(Shooter.getInstance());
    }
    @Override
    public void initialize() {
        Shooter.getInstance().setShooterVoltage(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
