// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // Constants for the arm connection info
  private static final int kArmRightID = 9;
  private static final int kArmLeftID = 10;
  private static final int kArmEncoderID = 0;

  private static final double kDefaultP = 0.0; // Proportional gain. Adjusts output based on current error.
  private static final double kDefaultI = 0.0; // Integral gain. Adjusts output based on accumulated error over time.
  private static final double kDefaultD = 0.0; // Derivative gain. Adjusts output based on rate of change of error.

  private static final double kDefaultSetpoint = 0.0; // The starting set point for the arm, About 90 degrees from the
                                                      // ground

  // Create a NetworkTable instance to enable the use of NetworkTables
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private CANSparkMax armR;
  private CANSparkMax armL;

  private PIDController pid;

  private DutyCycleEncoder armEncoder;

  // Create a single instance of the Arm class
  private static Arm instance = null;

  /**
   * Returns the instance of the Arm class. If the instance does not exist,
   * it creates a new one.
   * 
   * @return The instance of the Arm class
   */
  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

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

    // Create entries for P, I, and D values
    NetworkTableEntry pEntry = inst.getTable("Arm").getEntry("P");
    NetworkTableEntry iEntry = inst.getTable("Arm").getEntry("I");
    NetworkTableEntry dEntry = inst.getTable("Arm").getEntry("D");
    NetworkTableEntry setpointEntry = inst.getTable("Arm").getEntry("Setpoint");

    // Set the entries to be persistent
    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();
    setpointEntry.setPersistent();

    double p = inst.getTable("Arm").getEntry("P: ").getDouble(kDefaultP);
    double i = inst.getTable("Arm").getEntry("I: ").getDouble(kDefaultI);
    double d = inst.getTable("Arm").getEntry("D: ").getDouble(kDefaultD);
    double setpoint = inst.getTable("Arm").getEntry("Setpoint: ").getDouble(kDefaultSetpoint);

    pid = new PIDController(p, i, d);
    pid.setSetpoint(setpoint);

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
  }
/**
 * This method is called periodically and updates the PID controller with the current setpoint
 * and PID values from the network table.
 */
  @Override
  public void periodic() {
    // make shure pid values are updated from the network table
    pid.setP(inst.getTable("Arm").getEntry("P: ").getDouble(kDefaultP));
    pid.setI(inst.getTable("Arm").getEntry("I: ").getDouble(kDefaultI));
    pid.setD(inst.getTable("Arm").getEntry("D: ").getDouble(kDefaultD));

    // Update the PID controller with the current setpoint
    pid.setSetpoint(inst.getTable("Arm").getEntry("Setpoint: ").getDouble(kDefaultSetpoint));

    //Set motor power
    armR.set(pid.calculate(armEncoder.getAbsolutePosition()));
    armL.set(pid.calculate(armEncoder.getAbsolutePosition()));

  }

  /**
   * Sets the setpoint of the PID controller
   * 
   * @param point The new setpoint
   */
  public void setAngle(double point) {
    // Was setPoit : LOL ;)

    pid.setSetpoint(point);

    // update network table
    inst.getTable("Arm").getEntry("Setpoint: ").setDouble(point);

  }

  /**
   * @return The current angle of the arm
   */
  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }

  /**
   * @return The current setpoint of the PID controller
   */
  public double getSetpoint() {
    return pid.getSetpoint();
  }
}