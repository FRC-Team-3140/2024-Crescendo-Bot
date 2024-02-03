// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // Constants for the arm connection info
  private static final int kArmRightID = 9;
  private static final int kArmLeftID = 10;
  private static final int kArmEncoderID = 0;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private static Arm instance = null;

  private CANSparkMax armR;
  private CANSparkMax armL;

  private PIDController pid;

  private DutyCycleEncoder armEncoder;

  /** Creates a new Arm. */
  private Arm() {
    armR = new CANSparkMax(kArmRightID, MotorType.kBrushless);
    armL = new CANSparkMax(kArmLeftID, MotorType.kBrushless);

    armR.restoreFactoryDefaults();
    armR.setIdleMode(IdleMode.kBrake);
    armR.setInverted(false);
    armR.burnFlash();

    armL.restoreFactoryDefaults();
    armL.setIdleMode(IdleMode.kBrake);
    armL.setInverted(true);
    armL.burnFlash();

    pid = new PIDController(0, 0, 0);

    inst.getTable("Arm").getEntry("P: ").setDouble(pid.getP());
    inst.getTable("Arm").getEntry("I: ").setDouble(pid.getI());
    inst.getTable("Arm").getEntry("D: ").setDouble(pid.getD());

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
  }

  @Override
  public void periodic() {
    pid.setP(inst.getTable("Arm").getEntry("P: ").getDouble(0));
    pid.setI(inst.getTable("Arm").getEntry("I: ").getDouble(0));
    pid.setD(inst.getTable("Arm").getEntry("D: ").getDouble(0));
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public void setAngle(double point) {
    // Was setPoit : LOL ;)

    pid.setSetpoint(point);
  }

  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }
}
