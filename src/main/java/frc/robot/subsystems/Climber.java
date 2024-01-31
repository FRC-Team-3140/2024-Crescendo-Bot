package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    //Motors
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;

    //Limit Switches
    private DigitalInput rightUpperLimit;
    private DigitalInput rightLowerLimit;
    private DigitalInput leftUpperLimit;
    private DigitalInput leftLowerLimit;

    //Solenoids - Probably does not work
    private DigitalOutput leftSolenoid;
    private DigitalOutput rightSolenoid;
    
    //left motor CAN ID, right motor CAN ID, limit switches DIO port IDs, solenoid relay port IDs
    public Climber(int leftCANID, int rightCANID, int rightUpperLimitID,int rightLowerLimitID, int leftUpperLimitID, int leftLowerLimitID, int leftSolenoidID, int rightSolenoidID){
        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);

        rightUpperLimit = new DigitalInput(rightUpperLimitID);
        rightLowerLimit = new DigitalInput(rightLowerLimitID);
        leftUpperLimit = new DigitalInput(leftUpperLimitID);
        leftLowerLimit = new DigitalInput(leftLowerLimitID);

        leftSolenoid = new DigitalOutput(leftSolenoidID);
        rightSolenoid = new DigitalOutput(rightSolenoidID);


        //set motors
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(false);
        rightClimber.setInverted(false);

        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }

    /*
     * Raising and lowering climbers
     * Checks if the limit switch is pressed. If it is, sets motor to 0. If not, sets motor to a speed.
     * 
     */

    //raises the right climber
    public void raiseRight(){
        if (rightUpperLimit.get()) {
            // We are going up and top limit is tripped so stop
            rightClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            rightClimber.set(0); //change to an actual value later
        }
    }

    //raises the left climber
    public void raiseLeft(){
        if (leftUpperLimit.get()) {
            // We are going up and top limit is tripped so stop
            leftClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            leftClimber.set(0); //change to an actual value later
        }
    }

    //lowers the right climber
    public void lowerRight(){
        if (rightLowerLimit.get()) {
            // We are going up and top limit is tripped so stop
            rightClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            rightClimber.set(0); //change to an actual value later
        }
    }

    //lowers the left climber
    public void lowerLeft(){
        if (leftLowerLimit.get()) {
            // We are going up and top limit is tripped so stop
            leftClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            leftClimber.set(0); //change to an actual value later
        }
    }

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


