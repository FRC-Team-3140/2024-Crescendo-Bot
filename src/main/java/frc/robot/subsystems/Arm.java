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

  private static final String kSetpoint = "Setpoint";
  private static final String kArm = "Arm";
  private static final String kD = "D";
  private static final String kI = "I";
  private static final String kP = "P";
  private static final String kCurrentAngle = "CurrentAngle";
  // Constants for the arm connection info
  private static final int kArmRightID = 9;
  private static final int kArmLeftID = 10;
  private static final int kArmEncoderID = 0;

  private static final double kDefaultP = 0.0; // Proportional gain. Adjusts output based on current error.
  private static final double kDefaultI = 0.0; // Integral gain. Adjusts output based on accumulated error over time.
  private static final double kDefaultD = 0.0; // Derivative gain. Adjusts output based on rate of change of error.

  private static final double kDefaultSetpoint = 0.0; // The starting set point for the arm, About 90 degrees from the
                                                      // ground
  // limits on the setpoint
  private static final double kMaxSetpoint = 120.0; // Sligtly forward in amp position
  private static final double kMinSetpoint = 30.0; // On the ground in intake position

  // Absolute encoder angle offsets
  private static final double kArmEncoderOffset = 0.0; // The offset of the arm encoder from the zero position in
                                                       // degrees
                                                      

  // Create a NetworkTable instance to enable the use of NetworkTables
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private boolean has_error = false; // Disables the arm if the encoder is not connected

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
  public static synchronized Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

 /**
  * The constructor for the Arm class. It is private to prevent multiple instances
  * of the class from being created.
  */
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
    NetworkTableEntry pEntry = inst.getTable(kArm).getEntry(kP);
    NetworkTableEntry iEntry = inst.getTable(kArm).getEntry(kI);
    NetworkTableEntry dEntry = inst.getTable(kArm).getEntry(kD);
    NetworkTableEntry setpointEntry = inst.getTable(kArm).getEntry(kSetpoint);

    // Set the entries to be persistent
    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();
    setpointEntry.setPersistent();

    double p = inst.getTable(kArm).getEntry(kP).getDouble(kDefaultP);
    double i = inst.getTable(kArm).getEntry(kI).getDouble(kDefaultI);
    double d = inst.getTable(kArm).getEntry(kD).getDouble(kDefaultD);
    double setpoint = inst.getTable(kArm).getEntry(kSetpoint).getDouble(kDefaultSetpoint);

    pid = new PIDController(p, i, d);
    pid.setSetpoint(setpoint);

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
    // check that ArmEncoder is connected
    if (!armEncoder.isConnected()) {
      System.err.println("Arm Encoder not connected. Arm subsystem will not be initialized.");
      has_error = true;
    }
  }
/**
 * This method is called periodically and updates the PID controller with the current setpoint
 * and PID values from the network table.
 */
  @Override
  public void periodic() {
    // make shure pid values are updated from the network table
    pid.setP(inst.getTable(kArm).getEntry(kP).getDouble(kDefaultP));
    pid.setI(inst.getTable(kArm).getEntry(kI).getDouble(kDefaultI));
    pid.setD(inst.getTable(kArm).getEntry(kI).getDouble(kDefaultD));

    // Get the current setpoint in the network table. Check the limits and then 
    // update the PID controller with the current setpoint
    double setpoint = inst.getTable(kArm).getEntry(kSetpoint).getDouble(kDefaultSetpoint);
    if (setpoint > kMaxSetpoint) {
      setpoint = kMaxSetpoint;
    } else if (setpoint < kMinSetpoint) {
      setpoint = kMinSetpoint;
    }
    pid.setSetpoint(setpoint);

    //Set motor power
    if(has_error) {
      System.err.println("Arm Encoder not connected. Disabled.");
      armR.set(0);
      armL.set(0);
      return;
    }

    // set the arm motor power
    setPower(pid.calculate(getAngle()));

    // save the current arm angle to the network table
    inst.getTable(kArm).getEntry(kCurrentAngle).setDouble(getAngle());

  }

  /**
   * Sets the power of the arm motors
   * 
   * @param power
   */
  private void setPower(double power) {
    // check that the arm is not disabled
    if(has_error) {
      System.err.println("Arm Encoder not connected. Disabled.");
      armR.set(0);
      armL.set(0);
      return;
    }
    armR.set(power);
    armL.set(power);
  }  

  /**
   * Sets the setpoint of the PID controller
   * 
   * @param point The new setpoint
   */
  public void setAngle(double point) {
    // Was setPoit : LOL ;)

    // check that the setpoint is within the limits
    if (point > kMaxSetpoint) {
      point = kMaxSetpoint;
    } else if (point < kMinSetpoint) {
      point = kMinSetpoint;
    }

    // update network table
    inst.getTable(kArm).getEntry(kSetpoint).setDouble(point);
  }

  /**
   * @return The current angle of the arm
   */
  public double getAngle() {
    // This is the only place where the arm encoder is read
    return armEncoder.getAbsolutePosition() + kArmEncoderOffset;
  }

  /**
   * @return The current setpoint of the PID controller
   */
  public double getSetpoint() {
    return pid.getSetpoint();
  }
}