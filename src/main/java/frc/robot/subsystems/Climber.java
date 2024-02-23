package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Motors
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;
    public static Climber climber;

    // Solenoids - runs on PCM ports 0 and 1
    Solenoid leftSolenoid;
    Solenoid rightSolenoid;

    // CAN IDs
    private int leftCANID = 14;
    private int rightCANID = 15;
    // private int pcmCANID =

    // Relay ports
    private int leftSolenoidChannelID = 0;
    private int rightSolenoidChannelID = 4;

    // climber
    public Climber() {
        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);
        ;
        // electromagnetic push-pull solenoids running on the PCM.
        leftSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, leftSolenoidChannelID);
        rightSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, rightSolenoidChannelID);
        // .set(true) will pull the solenoids in. .set(false) will release the solenoids
        // to lock the climbers.

        // set motor settings
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(false);
        rightClimber.setInverted(false);

        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }

    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
            return climber;
        } else
            return climber;
    }

    /*
     * Raising and lowering climbers
     */
    // raises the left climber
    public void retractLeftSolenoid() {
        leftSolenoid.set(true);
    }

    public void raiseLeftMotor() {
        leftClimber.set(.2);
    }
    public void raiseLeft(){
        leftClimber.set(-.2);
        leftSolenoid.set(true);
        leftClimber.set(.2); //change to an actual value later
        
    }

    // raises the right climber
    public void raiseRight() {
        rightClimber.set(-.2);
        rightSolenoid.set(true);
        
        rightClimber.set(.2); //change to an actual value later
        
    }

    // lowers the left climber
    public void lowerLeft() {
        leftSolenoid.set(true);
        leftClimber.set(-.2); // change to an actual value later

    }

    // lowers the right climber
    public void lowerRight() {
        rightSolenoid.set(true);
        rightClimber.set(-.2); // change to an actual value later

    }

    // stops the left climber
    public void stopLeft() {
        leftClimber.set(0);
        leftSolenoid.set(false);
    }

    // stops the right climber
    public void stopRight() {
        rightClimber.set(0);
        rightSolenoid.set(false);
    }

    // These will probably never be used
    // raises both left and right climbers
    public void raiseBoth() {
        raiseLeft();
        raiseRight();
    }

    // lowers both left and right climbers
    public void lowerBoth() {
        lowerLeft();
        lowerRight();
    }
    public void retractRightSolenoid() {
       rightSolenoid.set(true);
    }


}
