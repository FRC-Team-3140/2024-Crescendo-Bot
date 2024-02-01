// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class IntakeShooter extends SubsystemBase {

    public  static IntakeShooter instance = null;
    public CANSparkMax intakeMotor = new CANSparkMax(20, MotorType.kBrushless);
    public CANSparkMax shooterA = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax shooterB = new CANSparkMax(8, MotorType.kBrushless);
    // private DigitalInput noteSensor;
    public RelativeEncoder encoderA = shooterA.getEncoder();
    public RelativeEncoder encoderB = shooterB.getEncoder();

    
    
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    SysIdRoutine routine = new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                shooterA.setVoltage(volts.in(Volts));
                shooterB.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterA.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterB.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));   


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
        intakeMotor.setVoltage(power);
    }

//Sets the speed of the shooter's motor, make sure one is negative and one is postive
    public void voltageDrive(double power) {
        shooterA.setVoltage(-power);
        shooterB.setVoltage(power);
    } 

    public void logMotors(){

    }

// Not filled out yet, will do when shooter/intake is done
    // public boolean getNoteSensor() {
    //     return noteSensor.get();
    // }

    @Override
    public void periodic() {
    }


}