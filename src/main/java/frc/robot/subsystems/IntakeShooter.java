// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeShooter extends SubsystemBase {

    public  static IntakeShooter instance = null;
    public CANSparkMax intakeMotor = new CANSparkMax(20, MotorType.kBrushless);
    private CANSparkMax shooterA = new CANSparkMax(9, MotorType.kBrushless);
    private CANSparkMax shooterB = new CANSparkMax(8, MotorType.kBrushless);
    // private DigitalInput noteSensor;

// Returns the instance
    public static synchronized IntakeShooter getInstance() {
        if (instance == null) {
            instance = new IntakeShooter();
        }
        return instance;
    }
// Sets the motor to neutral
    public  IntakeShooter() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

// Sets the speed of the intake motor through power
    public void intake(double power) {
        intakeMotor.set(power);
    }

//Sets the speed of the shooter's motor, make sure one is negative and one is postive
    public void shoot(double power) {
        shooterA.set(-power);
        shooterB.set(power);
    } 

// Not filled out yet, will do when shooter/intake is done
    // public boolean getNoteSensor() {
    //     return noteSensor.get();
    // }

    @Override
    public void periodic() {
    // SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));
    }
}


