// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeAndShooter extends SubsystemBase {

    private static IntakeAndShooter instance = null;
    private CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kUnbrushed);
    private CANSparkMax shooterA = new CANSparkMax(2, MotorType.kUnbrushed);
    private CANSparkMax shooterB = new CANSparkMax(3, MotorType.kUnbrushed);
    private DigitalInput noteSensor;

// Returns the instance
    public static synchronized IntakeAndShooter getInstance() {
        if (instance == null) {
            instance = new IntakeAndShooter();
        }
        return instance;
    }
// Sets the motor to neutral
    private IntakeAndShooter() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

// Sets the speed of the intake motor through power
    public void intake(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

// Sets the speed of the shooter's motor, make sure one is negative and one is postive
    public void shoot(double power) {
        shooterA.set(ControlMode.PercentOutput, -power);
        shooterB.set(ControlMode.PercentOutput, power);
    }

// Not filled out yet, will do when shooter/intake is done
    public boolean getNoteSensor() {
        return noteSensor.get();
    }

    @Override
    public void periodic() {

    }
}

