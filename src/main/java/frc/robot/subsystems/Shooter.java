package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.StateSpaceShooterHelp;

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
    private StateSpaceShooterHelp topController = new StateSpaceShooterHelp();
    private StateSpaceShooterHelp bottomController = new StateSpaceShooterHelp();

    private Shooter(){
        topShooter.restoreFactoryDefaults();
        topShooter.setIdleMode(IdleMode.kCoast);
        topShooter.setInverted(false);
        topShooter.setSmartCurrentLimit(30);
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
