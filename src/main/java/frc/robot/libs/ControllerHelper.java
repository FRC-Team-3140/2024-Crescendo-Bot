package frc.robot.libs;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ControllerHelper extends XboxController {
    public ControllerHelper(int port) {
        super(port);
    }

    double deadband = .07;
    
    @Override
    public double getLeftX() {
        if (Math.abs(super.getLeftX()) > deadband) {
            if (super.getLeftX() > 0)
                return Math.pow(super.getLeftX(), 2);
            else
                return -Math.pow(super.getLeftX(), 2);

        } else {
            return 0;
        }
    }

    @Override
    public double getRightX() {
        if (Math.abs(super.getRightX()) > deadband) {
            if (super.getRightX() > 0)
                return Math.pow(super.getRightX(), 2);
            else
                return -Math.pow(super.getRightX(), 2);
        } else {
            return 0;
        }
    }

    @Override
    public double getLeftY() {
        if (Math.abs(super.getLeftY()) > deadband) {
            if (super.getLeftY() > 0)
                return Math.pow(super.getLeftY(), 2);
            else
                return -Math.pow(super.getLeftY(), 2);

        } else {
            return 0;
        }
    }

    @Override
    public double getRightY() {
        if (Math.abs(super.getRightY()) > deadband) {
            if (super.getRightY() > 0)
                return Math.pow(super.getRightY(), 2);
            else
                return -Math.pow(super.getRightY(), 2);
        } else {
            return 0;
        }
    }

    public Command setRumble(){
            return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(()-> {setRumble(RumbleType.kBothRumble, 1);}), 
            new WaitCommand(1)), new InstantCommand(()-> {setRumble(RumbleType.kBothRumble, 0);}));
    }
}
