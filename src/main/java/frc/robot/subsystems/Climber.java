package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climber extends SubsystemBase{
    //Motors
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;

    //Solenoids - runs on PCM ports 0 and 1
    Solenoid leftSolenoid;
    Solenoid rightSolenoid;

    //Limit Switches
    DigitalInput leftLimit;
    DigitalInput rightLimit;

    //CAN IDs
    private int leftCANID = 14;
    private int rightCANID = 15;
    // private int pcmCANID = 

    //Relay ports
    private int leftSolenoidChannelID = 0;
    private int rightSolenoidChannelID = 4;
    
    //climber
    static Climber climber = new Climber();
    
    public Climber(){
        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);
;
        //electromagnetic push-pull solenoids running on the PCM. 
        leftSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, leftSolenoidChannelID); 
        rightSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, rightSolenoidChannelID);
        //.set(true) will pull the solenoids in. .set(false) will release the solenoids to lock the climbers.

        //Limit Switch DIO ports
        leftLimit = new DigitalInput(0);
        rightLimit = new DigitalInput(1);

        //set motor settings
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(false);
        rightClimber.setInverted(false);

        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }
    public static Climber getInstance(){
        return climber;
    }

    /*
     * Raising and lowering climbers
     */
    //raises the left climber
    public void raiseLeft(){
        leftSolenoid.set(true);
        leftClimber.set(1); //change to an actual value later
        
    }

    //raises the right climber
    public void raiseRight(){
        rightSolenoid.set(true);
        rightClimber.set(.2); //change to an actual value later
        
    }

    //lowers the left climber
    public void lowerLeft(){
        if(!leftLimit.get()){
            leftSolenoid.set(true);
            leftClimber.set(-.2); //change to an actual value later
        }else{
            leftClimber.set(0);
        }
    }

    //lowers the right climber
    public void lowerRight(){
        if(!rightLimit.get()){
            rightSolenoid.set(true);
            rightClimber.set(-.2); //change to an actual value later
        }else{
            rightClimber.set(0);
        }
    }

    //stops the left climber
    public void stopLeft(){
        leftClimber.set(0);
        leftSolenoid.set(false);
    }
    public SequentialCommandGroup increaseLeftHeight(){
        return new SequentialCommandGroup(new InstantCommand(this::lowerLeft), new WaitCommand(.1), new InstantCommand(this::raiseLeft));
    }
    public SequentialCommandGroup increaseRightHeight(){
        return new SequentialCommandGroup(new InstantCommand(this::lowerRight), new WaitCommand(.1), new InstantCommand(this::raiseRight));
    }
    //stops the right climber
    public void stopRight(){
        rightClimber.set(0);
        rightSolenoid.set(false);
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


