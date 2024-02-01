// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeShooter extends SubsystemBase {

    private static IntakeShooter instance = null;
    private CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax shooterA = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax shooterB = new CANSparkMax(3, MotorType.kBrushless);
    private DigitalInput noteSensor;

    public IntakeShooter(){
        intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
        shooterA = new CANSparkMax(2, MotorType.kBrushless);
        shooterB = new CANSparkMax(3, MotorType.kBrushless);
        noteSensor = new DigitalInput(0); //Random number, once we put it on there we will update it

        intakeMotor.setIdleMode(IdleMode.kBrake);

        
    }

// Returns the instance
    public static IntakeShooter getInstance() {
        if (instance == null) {
            instance = new IntakeShooter();
        }
        return instance;
    }

// Sets the speed of the intake motor through power
    public void intake(double power) {
        intakeMotor.set(power);
    }

// Sets the speed of the shooter's motor, make sure one is negative and one is postive
    public void shoot(double power) {
        shooterA.set(-power);
        shooterB.set(power);
    }

// Not filled out yet, will do when shooter/intake is done
    public boolean getNoteSensor() {
        return noteSensor.get();
    }

    @Override
    public void periodic() {

    }
}

