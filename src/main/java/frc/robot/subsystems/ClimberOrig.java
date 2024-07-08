package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    // Motors
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    // Solenoids - runs on PCM ports 0 and 1
    Solenoid leftSolenoid;
    Solenoid rightSolenoid;

    // Limit Switches
    DigitalInput leftLimit;
    DigitalInput rightLimit;

    // CAN IDs
    private int leftCANID = 14;
    private int rightCANID = 15;
    // private int pcmCANID =

    // Relay ports
    private int leftSolenoidChannelID = 0;
    private int rightSolenoidChannelID = 5;

    //Constants
    private double raiseSpeed = .5;
    private double lowerSpeed = .75;
    // climber
    static Climber climber = new Climber();
    BooleanSupplier leftLimitSwitchPressed;
    BooleanSupplier rightLimitSwitchPressed;
    BooleanSupplier leftReachedTop;
    BooleanSupplier rightReachedTop;
    public Climber() {

        leftClimber = new CANSparkMax(leftCANID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(rightCANID, MotorType.kBrushless);
        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();
        // electromagnetic push-pull solenoids running on the PCM.
        leftSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, leftSolenoidChannelID);
        rightSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, rightSolenoidChannelID);
        // .set(true) will pull the solenoids in. .set(false) will release the solenoids
        // to lock the climbers.

        // Limit Switch DIO ports
        leftLimit = new DigitalInput(4);
        rightLimit = new DigitalInput(5);

        leftLimitSwitchPressed = () -> leftLimit.get();
        rightLimitSwitchPressed = () -> rightLimit.get();
        leftReachedTop = () -> encoderValues()[1] > Constants.topClimberPosition+1.3;
        rightReachedTop = () -> encoderValues()[0] > Constants.topClimberPosition;

        // set motor settings
        leftClimber.restoreFactoryDefaults();
        leftClimber.setSmartCurrentLimit(30);
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.restoreFactoryDefaults();
        rightClimber.setSmartCurrentLimit(30);
        rightClimber.setIdleMode(IdleMode.kBrake);
        leftClimber.setInverted(false);
        rightClimber.setInverted(false);
        //quarter inch, 15:1 gear ratio
        new Trigger(leftLimitSwitchPressed).onTrue(new InstantCommand(this::stopLeft).alongWith(new InstantCommand(this::resetLeft)));
        new Trigger(rightLimitSwitchPressed).onTrue(new InstantCommand(this::stopRight).alongWith(new InstantCommand(this::resetRight)));
        new Trigger(leftReachedTop).onTrue(new InstantCommand(this::stopLeft));
        new Trigger(rightReachedTop).onTrue(new InstantCommand(this::stopRight));
        leftClimber.burnFlash();
        rightClimber.burnFlash();

    } 

    public static Climber getInstance() {
        return climber;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftPosition", encoderValues()[1]);
        SmartDashboard.putNumber("RightPosition", encoderValues()[0]);
    }

    /*
     * Raising and lowering climbers
     */
    // raises the left climber
    private void raiseLeft() {
        leftSolenoid.set(true);
        leftClimber.set(raiseSpeed); //change to an actual value later
    }

    // raises the right climber
    private void raiseRight() {
        rightSolenoid.set(true);
        rightClimber.set(raiseSpeed); //change to an actual value later
        
    }

    public void lowerLeftForRaising() {
        leftSolenoid.set(true);
        leftClimber.set(-.1); 
    }

    public void lowerRightForRaising() {
        rightSolenoid.set(true);
        rightClimber.set(-.1); 
    }

    // lowers the left climber
    public void lowerLeft() {
        if (!leftLimit.get()) {
            leftSolenoid.set(true);
            leftClimber.set(-lowerSpeed); //change to an actual value later
        }else{
            leftClimber.set(0);
        }
    }

    // lowers the right climber
    public void lowerRight() {
        if (!rightLimit.get()) {
            rightSolenoid.set(true);
            rightClimber.set(-lowerSpeed); //change to an actual value later
        }else{
            rightClimber.set(0);
        }
    }

    // stops the left climber
    public void stopLeft() {
        leftClimber.set(0);
        leftSolenoid.set(false);
    }
    public SequentialCommandGroup increaseLeftHeight(){
        if(leftReachedTop.getAsBoolean()){
            return new SequentialCommandGroup();
        }
        return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(this::lowerLeftForRaising), new WaitCommand(0.5)), new InstantCommand(this::raiseLeft));
    }
    public SequentialCommandGroup increaseRightHeight(){
        if(rightReachedTop.getAsBoolean()/*rightLimitSwitchPressed.getAsBoolean()*/){
            return new SequentialCommandGroup();
        }
        return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(this::lowerRightForRaising), new WaitCommand(0.5)),  new InstantCommand(this::raiseRight));
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

    public void stopBoth() {
        stopLeft();
        stopRight();
    }

    // lowers both left and right climbers
    public void lowerBoth() {
        lowerLeft();
        lowerRight();
    }

    public void retractRightSolenoid(){
        rightSolenoid.set(true);
    }

        
    public void resetLeft(){
        leftClimber.getEncoder().setPosition(0);
    }
    public void resetRight(){
        rightClimber.getEncoder().setPosition(0);
    }

    public double[] encoderValues(){
        double[] values = {rightEncoder.getPosition(), leftEncoder.getPosition()};
        return values;
    }
    public boolean bothLimitSwitchesPressed(){
        return leftLimitSwitchPressed.getAsBoolean() && rightLimitSwitchPressed.getAsBoolean();
    }
    public boolean bothReachedTop(){
        return leftReachedTop.getAsBoolean() && rightReachedTop.getAsBoolean();
    }

}