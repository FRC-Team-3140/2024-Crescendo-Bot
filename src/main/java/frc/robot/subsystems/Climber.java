package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;

    private DigitalInput rightUpperLimit;
    private DigitalInput rightLowerLimit;
    private DigitalInput leftUpperLimit;
    private DigitalInput leftLowerLimit;
    

    public Climber(int leftCANID, int rightCANID, int rightUpperLimitID,int rightLowerLimitID, int leftUpperLimitID, int leftLowerLimitID){
        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);

        rightUpperLimit = new DigitalInput(rightUpperLimitID);
        rightLowerLimit = new DigitalInput(rightLowerLimitID);
        leftUpperLimit = new DigitalInput(leftUpperLimitID);
        leftLowerLimit = new DigitalInput(leftLowerLimitID);


        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(false);
        rightClimber.setInverted(false);

        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }


    public void raiseRight(){
        if (rightUpperLimit.get()) {
            // We are going up and top limit is tripped so stop
            rightClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            rightClimber.set(0); //change to an actual value later
        }
    }

    public void raiseLeft(){
        if (leftUpperLimit.get()) {
            // We are going up and top limit is tripped so stop
            leftClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            leftClimber.set(0); //change to an actual value later
        }
    }

    public void lowerRight(){
        if (rightLowerLimit.get()) {
            // We are going up and top limit is tripped so stop
            rightClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            rightClimber.set(0); //change to an actual value later
        }
    }

    public void lowerLeft(){
        if (leftLowerLimit.get()) {
            // We are going up and top limit is tripped so stop
            leftClimber.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            leftClimber.set(0); //change to an actual value later
        }
    }

    public void raiseBoth(){
        raiseLeft();
        raiseRight();
    }

    public void lowerBoth(){
        lowerLeft();
        lowerRight();
    }
}


