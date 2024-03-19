package frc.robot.commands.L1Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberToPosition extends Command {
    double position;
    Climber climber;
    double deadband = .7;
    double[] initialValue;
    

    public SetClimberToPosition(double position){
        climber = Climber.getInstance();
        this.position = position;
        addRequirements(climber);
        
    }

    @Override
    public void initialize() {
        if(climber.encoderValues()[0] > position){
            climber.lowerRight();
        }else if(climber.encoderValues()[0] < position){
            climber.increaseRightHeight().schedule();
        }
        if(climber.encoderValues()[1] > position){
            climber.lowerLeft();
        }else if(climber.encoderValues()[1] < position){
            climber.increaseLeftHeight().schedule();
        }
    }

    public void execute(){
        if(Math.abs(climber.encoderValues()[0] - position) < deadband){
            climber.stopRight();
        }
        if(Math.abs(climber.encoderValues()[1] - position) < deadband){
            climber.stopLeft();
        }
    }
    @Override
    public boolean isFinished() {
        return Math.abs(climber.encoderValues()[0] - position) < deadband && Math.abs(climber.encoderValues()[1] - position) < deadband;
    }

    
}
