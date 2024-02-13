package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    //Motors
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;

    //Solenoids - Probably does not work
    private DigitalOutput leftSolenoid;
    private DigitalOutput rightSolenoid;

    //CAN IDs
    private int leftCANID = 14;
    private int rightCANID = 15;

    //Relay ports
    private int leftSolenoidID = 0;
    private int rightSolenoidID = 1;
    
    //climber
    public Climber(){
        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);
;
        //electromagnetic push-pull solenoids
        leftSolenoid = new DigitalOutput(leftSolenoidID);
        rightSolenoid = new DigitalOutput(rightSolenoidID);


        //set motor settings
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(false);
        rightClimber.setInverted(false);

        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }

    /*
     * Raising and lowering climbers
     */
    //raises the left climber
    public void raiseLeft(){
        
        leftClimber.set(.1); //change to an actual value later
        
    }

    //raises the right climber
    public void raiseRight(){
        
        rightClimber.set(.1); //change to an actual value later
        
    }

    //lowers the left climber
    public void lowerLeft(){
        leftClimber.set(-.1); //change to an actual value later
        
    }

    //lowers the right climber
    public void lowerRight(){
        rightClimber.set(-.1); //change to an actual value later
        
    }

    
    public void stopLeft(){
        leftClimber.set(0);
    }

    public void stopRight(){
        rightClimber.set(0);
    }

    //These will probably never be used
    //raises both left and right climbers
    public void raiseBoth(){
        raiseLeft();
        raiseRight();
    }

    //lowers both left and right climbers
    public void lowerBoth(){
        lowerLeft();
        lowerRight();
    }

}


