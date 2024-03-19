package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.StateSpaceShooterHelp;

/* 
 * TODO: Apply Copilot's suggestions:
 * 
 * 1. Remove Duplicate Code: Eliminate the duplicate line 
 *    'bottomShooter.setInverted(true);'.
 * 
 * 2. Singleton Pattern: Make the constructor private to prevent 
 *    instantiation from outside the class.
 * 
 * 3. Constants: Define current limit and velocity conversion factor 
 *    as constants for easier management and modification.
 * 
 * 4. Access Modifiers: Make 'topShooter', 'topEncoder', 'bottomShooter', 
 *    'bottomEncoder', 'topController', and 'bottomController' private. 
 *    Provide public getter methods for controlled access.
 * 
 * 5. Error Checking: Add null checks for controllers and encoders in 
 *    the 'periodic()' method.
 */

public class Shooter extends SubsystemBase {

    public static Shooter instance = new Shooter();

    public CANSparkMax topShooter = new CANSparkMax(9, MotorType.kBrushless);
    public RelativeEncoder topEncoder = topShooter.getEncoder();

    /**
     * Flywheel 2 on the shooter.
     */
    public CANSparkMax bottomShooter = new CANSparkMax(10, MotorType.kBrushless);
    /**
     * Relative Encoder from flywheel 2's neo.
     */
    public RelativeEncoder bottomEncoder = bottomShooter.getEncoder();
    private StateSpaceShooterHelp topController = new StateSpaceShooterHelp(); //TODO: No.  Too complicated.  Just use a PID controller. Replace all this extra code and physical modeling with 2-3 lines of PID code.
    private StateSpaceShooterHelp bottomController = new StateSpaceShooterHelp();

    /*  TODO: I would recommend if you are going to try complex manipulation 
    * of the shooter speed you should do it in commands as well as more complex 
    * distance shooting behaviors.  You can easily create alternative commands
    * and test them side by side by using different buttons.
    *
    * As a bonus.  They are easier to isolate with try catch and if one fails.
    * you can easily revert to an easier and simpler version.
    *
    * Create the simple commands first and then build on them with more complexity.  
    *
    * Keep the subsystem complexity low.
    */

    private Shooter(){
        topShooter.restoreFactoryDefaults();
        topShooter.setIdleMode(IdleMode.kCoast);
        topShooter.setInverted(false);
        topShooter.setSmartCurrentLimit(30); // TODO: Current limits may be ok.  This is where we need power.  Please test.
        topShooter.burnFlash();
        topEncoder.setVelocityConversionFactor(42);

        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setInverted(true);
        bottomShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setInverted(true);
        bottomShooter.setSmartCurrentLimit(30);
        bottomShooter.burnFlash();
        bottomEncoder.setVelocityConversionFactor(42);
    }
    @Override
    public void periodic() {
        topController.correct(VecBuilder.fill(topEncoder.getVelocity()));
        topController.predict();
        
        bottomController.correct(VecBuilder.fill(topEncoder.getVelocity()));
        bottomController.predict();
        
    }

    public static Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    public void setShooterSpeed(double speed){
        topController.setSpeed(speed);
        double nextVoltage = topController.getVoltage();
        topShooter.setVoltage(nextVoltage);
    }
    public void setShooterVoltage(double voltage) {
        topShooter.setVoltage(voltage);
        bottomShooter.setVoltage(-voltage);
    }
    public void setShooterVoltageTop(double voltage) {
        topShooter.setVoltage(voltage+.15);
    }
    public void setShooterVoltageBottom(double voltage) {
        bottomShooter.setVoltage(-voltage);
    }
    public double getTopShooterSpeed() {
        return topEncoder.getVelocity();
    }
    public double getBottomShooterSpeed() {
        return bottomEncoder.getVelocity();
    }
    public double getShooterSpeed() {
        return Math.min(getBottomShooterSpeed(), getTopShooterSpeed());
    }

}
