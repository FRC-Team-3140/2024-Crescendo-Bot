import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

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

/**
 * The `Climber` class represents a subsystem that controls the climbers of a
 * robot. It provides methods to raise, lower, and stop the left and right 
 * climbers independently.The class also includes limit switch detection, 
 * encoder position tracking, and solenoid control.
 */
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

    // Constants
    private double raiseSpeed = .5;
    private double lowerSpeed = .75;
    // climber
    static Climber climber = new Climber();
    BooleanSupplier leftLimitSwitchPressed;
    BooleanSupplier rightLimitSwitchPressed;
    BooleanSupplier leftReachedTop;
    BooleanSupplier rightReachedTop;

    /**
     * Constructs a new instance of the Climber class.
     * Initializes the left and right climbers, encoders, solenoids, limit switches,
     * and sets the motor settings.
     */
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
        leftReachedTop = () -> encoderValues()[1] > Constants.topClimberPosition + 1.3;
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
        // quarter inch, 15:1 gear ratio
        new Trigger(leftLimitSwitchPressed)
                .onTrue(new InstantCommand(this::stopLeft).alongWith(new InstantCommand(this::resetLeft)));
        new Trigger(rightLimitSwitchPressed)
                .onTrue(new InstantCommand(this::stopRight).alongWith(new InstantCommand(this::resetRight)));
        new Trigger(leftReachedTop).onTrue(new InstantCommand(this::stopLeft));
        new Trigger(rightReachedTop).onTrue(new InstantCommand(this::stopRight));
        leftClimber.burnFlash();
        rightClimber.burnFlash();

    }

    /**
     * Returns the singleton instance of the Climber subsystem.
     *
     * @return the singleton instance of the Climber subsystem
     */
    public static Climber getInstance() {
        return climber;
    }

    /**
     * This method is called periodically by the robot program.
     * It updates the SmartDashboard with the current encoder values.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftPosition", encoderValues()[1]);
        SmartDashboard.putNumber("RightPosition", encoderValues()[0]);
    }


    /**
     * Raises the left side of the climber.
     */
    private void raiseLeft() {
        leftSolenoid.set(true);
        leftClimber.set(raiseSpeed); // change to an actual value later
    }

    /**
     * Raises the right side of the climber.
     */
    private void raiseRight() {
        rightSolenoid.set(true);
        rightClimber.set(raiseSpeed); // change to an actual value later

    }

    /**
     * Lowers the left side of the climber for raising the robot.
     */
    public void lowerLeftForRaising() {
        leftSolenoid.set(true);
        leftClimber.set(-.1);
    }

    /**
     * Lowers the right side of the climber for raising the robot.
     */
    public void lowerRightForRaising() {
        rightSolenoid.set(true);
        rightClimber.set(-.1);
    }

    /**
     * Lowers the left side of the climber.
     * If the left limit switch is not triggered, the left solenoid is set to true and the left climber motor is set to a negative value.
     * Otherwise, the left climber motor is set to 0.
     */
    public void lowerLeft() {
        if (!leftLimit.get()) {
            leftSolenoid.set(true);
            leftClimber.set(-lowerSpeed); // change to an actual value later
        } else {
            leftClimber.set(0);
        }
    }

    /**
     * Lowers the right climber.
     * If the right limit switch is not triggered, the right solenoid is set to true and the right climber is set to a negative value at a lower speed.
     * If the right limit switch is triggered, the right climber is set to 0.
     */
    public void lowerRight() {
        if (!rightLimit.get()) {
            rightSolenoid.set(true);
            rightClimber.set(-lowerSpeed); // change to an actual value later
        } else {
            rightClimber.set(0);
        }
    }

    /**
     * Stops the left climber motor and solenoid.
     */
    public void stopLeft() {
        leftClimber.set(0);
        leftSolenoid.set(false);
    }

    /**
     * Increases the height of the left climber.
     * If the left climber has already reached the top, no commands will be executed.
     * Otherwise, it will execute a sequence of commands to lower the left climber for raising and then raise the left climber.
     *
     * @return A SequentialCommandGroup containing the commands to increase the left climber's height.
     */
    public SequentialCommandGroup increaseLeftHeight() {
        if (leftReachedTop.getAsBoolean()) {
            return new SequentialCommandGroup();
        }
        return new SequentialCommandGroup(
                new ParallelCommandGroup(new InstantCommand(this::lowerLeftForRaising), new WaitCommand(0.5)),
                new InstantCommand(this::raiseLeft));
    }

    /**
     * Increases the height of the right climber.
     * If the right climber has reached the top, no commands will be executed.
     * Otherwise, it will execute a sequence of commands to lower the right climber for raising
     * and then raise the right climber.
     *
     * @return A SequentialCommandGroup containing the commands to increase the right climber's height.
     */
    public SequentialCommandGroup increaseRightHeight() {
        if (rightReachedTop.getAsBoolean()/* rightLimitSwitchPressed.getAsBoolean() */) {
            return new SequentialCommandGroup();
        }
        return new SequentialCommandGroup(
                new ParallelCommandGroup(new InstantCommand(this::lowerRightForRaising), new WaitCommand(0.5)),
                new InstantCommand(this::raiseRight));
    }

    /**
     * Stops the right climber motor and solenoid.
     */
    public void stopRight() {
        rightClimber.set(0);
        rightSolenoid.set(false);
    }

        // These will probably never be used


    /**
     * Raises both the left and right climbers.
     */
    public void raiseBoth() {
    // raises both left and right climbers
    raiseLeft();
        raiseRight();
    }

    /**
     * Stops both the left and right climber mechanisms.
     */
    public void stopBoth() {
        stopLeft();
        stopRight();
    }

    /**
     * Lowers both sides of the climber.
     */
    public void lowerBoth() {
        lowerLeft();
        lowerRight();
    }

    /**
     * Retracts the right solenoid of the climber subsystem.
     */
    public void retractRightSolenoid() {
        rightSolenoid.set(true);
    }

    /**
     * Resets the position of the left climber encoder to zero.
     */
    public void resetLeft() {
        leftClimber.getEncoder().setPosition(0);
    }

    /**
     * Resets the position of the right climber's encoder to zero.
     */
    public void resetRight() {
        rightClimber.getEncoder().setPosition(0);
    }

    /**
     * Returns an array containing the positions of the right and left encoders.
     *
     * @return an array of type double containing the positions of the right and left encoders
     */
    public double[] encoderValues() {
        double[] values = { rightEncoder.getPosition(), leftEncoder.getPosition() };
        return values;
    }

    /**
     * Checks if both limit switches are pressed.
     * 
     * @return true if both limit switches are pressed, false otherwise
     */
    public boolean bothLimitSwitchesPressed() {
        return leftLimitSwitchPressed.getAsBoolean() && rightLimitSwitchPressed.getAsBoolean();
    }

    /**
     * Checks if both sides of the climber have reached the top.
     * 
     * @return true if both sides have reached the top, false otherwise
     */
    public boolean bothReachedTop() {
        return leftReachedTop.getAsBoolean() && rightReachedTop.getAsBoolean();
    }

}