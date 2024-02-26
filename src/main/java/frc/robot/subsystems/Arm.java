// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // Constants for the PID controller
  private static final String kNTP = "P";
  private static final String kNTI = "I";
  private static final String kNTD = "D";

  // Constants for the arm state
  private static final String kNTCurrentAngle = "CurrentAngle";
  private static final String kNTIsEnabled = "IsEnabled";
  private static final String kNTErrorCode = "ErrorCode";

  // Constants for the arm power
  private static final String kNTForwardParam = "ForwardParam";
  private static final String kForwardPower = "ForwardPower";
  private static final String kNTPidPower = "PidPower";

  // Constants for the arm setpoint and motor speed
  private static final String kNTSetpoint = "Setpoint";
  private static final String kNTMotorSpeed = "MotorSpeed";

  // Constant for the arm network table
  private static final String kNTArm = "Arm";

  // Constants for the arm connection info
  private static final int kArmRightID = 13;
  private static final int kArmLeftID = 12;
  private static final int kArmEncoderID = 1;

  // Constants for the arm motor configuration
  private static final int kMotorCurrentLimit = 40; // The current limit for the arm motors
  private static final boolean kArmRightReversed = true; // Motor direction for right arm
  private static final boolean kArmLeftReversed = false; // Motor direction for left arm
  private static final IdleMode kEnabledMotorMode = IdleMode.kBrake; // Motor mode when enabled
  private static final IdleMode kDisabledMotorMode = IdleMode.kBrake; // Motor mode when disabled

  // Constants for the PID controller
  private static final double kDefaultP = .25; // Proportional gain
  private static final double kDefaultI = 0.0; // Integral gain
  private static final double kDefaultD = 0.003; // Derivative gain

  // Constants for the arm setpoint
  private static final double kDefaultSetpoint = 0.0; // The starting set point for the arm
  private static final double kMaxSetpoint = 94.0; // Maximum setpoint; Test again with Amp
  private static final double kMinSetpoint = 8.0; // Minimum setpoint

  // Favorite setpoints
  public static final double kSetpointShoot = 14.0; // The setpoint for shooting
  public static final double kSetpoiintIntakeDown = 8.0; // The setpoint for intaking
  public static final double kSetpointIntakeReady = 28.0; // The ready for intake but off the ground for movement and protection
  public static final double kSetpointAmp = 94.0; // The ready for intake but off the ground for movement and protection
  public static final double kSetpointMove = 60.0; // The ready for intake but off the ground for movement and
                                                   // protection

  // Constants for the arm control
  private static final double kDefaultForwardParam = .325; // The default forward control parameter
  private static final double kArmEncoderOffset = -152; // The offset of the arm encoder from the zero position //
                                                        // degrees

  // Create a NetworkTable instance to enable the use of NetworkTables
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private boolean is_disabled = false; // Disables the arm if the encoder is not connected

  private CANSparkMax armR;
  private CANSparkMax armL;

  private PIDController pid;

  private DutyCycleEncoder armEncoder;

  private InterpolatingDoubleTreeMap angleInterpolator;

  private double fcp = kDefaultForwardParam;

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
   * The constructor for the Arm class. It is private to prevent multiple
   * instances
   * of the class from being created.
   */
  private Arm() {
    armR = new CANSparkMax(kArmRightID, MotorType.kBrushless);
    armL = new CANSparkMax(kArmLeftID, MotorType.kBrushless);

    armR.restoreFactoryDefaults();
    armR.setIdleMode(kDisabledMotorMode);
    armR.setInverted(kArmRightReversed);
    armR.setSmartCurrentLimit(kMotorCurrentLimit);
    armR.burnFlash();

    armL.restoreFactoryDefaults();
    armL.setIdleMode(kDisabledMotorMode);
    armL.setInverted(kArmLeftReversed);
    armR.setSmartCurrentLimit(kMotorCurrentLimit);
    armL.burnFlash();

    // Create entries for P, I, and D values
    NetworkTableEntry pEntry = inst.getTable(kNTArm).getEntry(kNTP);
    NetworkTableEntry iEntry = inst.getTable(kNTArm).getEntry(kNTI);
    NetworkTableEntry dEntry = inst.getTable(kNTArm).getEntry(kNTD);
    NetworkTableEntry setpointEntry = inst.getTable(kNTArm).getEntry(kNTSetpoint);
    NetworkTableEntry fcpEntry = inst.getTable(kNTArm).getEntry(kNTForwardParam);

    // Set the entries to be persistent
    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();
    setpointEntry.setPersistent();
    fcpEntry.setPersistent();

    double p = inst.getTable(kNTArm).getEntry(kNTP).getDouble(kDefaultP);
    double i = inst.getTable(kNTArm).getEntry(kNTI).getDouble(kDefaultI);
    double d = inst.getTable(kNTArm).getEntry(kNTD).getDouble(kDefaultD);
    double setpoint = inst.getTable(kNTArm).getEntry(kNTSetpoint).getDouble(kDefaultSetpoint);
    fcp = inst.getTable(kNTArm).getEntry(kNTForwardParam).getDouble(kDefaultForwardParam);

    pid = new PIDController(p, i, d);
    pid.setSetpoint(setpoint);

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
    encoderConnected();

    angleInterpolator = new InterpolatingDoubleTreeMap();//Add your inverseInterpolator, interpolator, and comparator here
    angleInterpolator.put(1.4605, 14.0); // 14 Degrees and 42 inches measured to the inside of the bot perimiter
    angleInterpolator.put(1.700, 21.0);
    angleInterpolator.put(1.9685, 27.5);
    angleInterpolator.put(2.4003, 34.0);
    angleInterpolator.put(3.0048, 37.8);
    angleInterpolator.put(3.2385, 38.4);
    angleInterpolator.put(3.4163, 40.4);
    angleInterpolator.put(3.5146, 40.7);
    angleInterpolator.put(3.9116, 41.6);
    // angleInterpolator.put()

    
    // angleInterpolator = new InterpolatingDoubleTreeMap();//Add your inverseInterpolator, interpolator, and comparator here
    // angleInterpolator.put(1.35, 13.0); // 14 Degrees and 42 inches measured to the inside of the bot perimiter
    // angleInterpolator.put(2.42, 21.0);
    // angleInterpolator.put(3.14, 27.0);

    // Set the arm to disabled by default.
    disable();
  }

  private void encoderConnected() {
    // check that ArmEncoder is connected
    if (!armEncoder.isConnected()) {
      inst.getTable(kNTArm).getEntry(kNTErrorCode).setString("Arm encoder not connected");
      is_disabled = true;
    }
  }

  /**
   * This method is called periodically and updates the PID controller with the
   * current setpoint
   * and PID values from the network table.
   */
  @Override
  public void periodic() {
    // make sure pid values are updated from the network table
    pid.setP(inst.getTable(kNTArm).getEntry(kNTP).getDouble(kDefaultP));
    pid.setI(inst.getTable(kNTArm).getEntry(kNTI).getDouble(kDefaultI));
    pid.setD(inst.getTable(kNTArm).getEntry(kNTI).getDouble(kDefaultD));

    // Get the current setpoint in the network table. Check the limits and then
    // update the PID controller with the current setpoint
    double setpoint = inst.getTable(kNTArm).getEntry(kNTSetpoint).getDouble(kDefaultSetpoint);

    if (setpoint > kMaxSetpoint) {
      setpoint = kMaxSetpoint;
    } else if (setpoint < kMinSetpoint) {
      setpoint = kMinSetpoint;
    }

    // Update the setpoint incase it has changed
    inst.getTable(kNTArm).getEntry(kNTSetpoint).setDouble(setpoint);

    // set the arm motor power
    updatePower(pid.calculate(getAngle()));

  }

  public void setArmToAngle(double setPoint) {
    double angle = getAngle();
    double setpoint = setPoint;

    // The forward controll needed is proportional to the cosine of the angle
    double forward_power = kDefaultForwardParam
        * Math.cos(Math.toRadians(angle));

    pid.setSetpoint(setpoint);
    double pid_power = pid.calculate(angle);

    double voltage = pid_power + forward_power;

    // Update the network table with the forward and PID power
    inst.getTable(kNTArm).getEntry(kNTSetpoint).setDouble(setpoint);
    inst.getTable(kNTArm).getEntry(kForwardPower).setDouble(forward_power);
    inst.getTable(kNTArm).getEntry(kNTPidPower).setDouble(pid_power);
    inst.getTable(kNTArm).getEntry(kNTMotorSpeed).setDouble(voltage);

    armR.setVoltage(voltage);
    armL.setVoltage(voltage);
  }

  /**
   * Sets the arm to shoot at a specific distance
   * 
   * @param distance The distance to shoot at
   * @return The setpoint angle for the arm
   */
  public double setArmToShootDistance(double distance) {
    double interpolatedAngle = angleInterpolator.get(distance);
    setArmToAngle(interpolatedAngle);
    return interpolatedAngle;
  }

  /**
   * Sets the power of the arm motors
   * 
   * @param power
   */
  private void updatePower(double power) {
    // check that the arm is not disabled
    if (is_disabled || !armEncoder.isConnected()) {
      // System.err.println("Arm Encoder not connected. Disabled.");
      armR.setVoltage(0);
      armL.setVoltage(0);
      return;
    }

    // Add a forward control component based on the current angle
    // This is to prevent the arm from falling under gravity
    // An angle of 0 level with the ground
    // An angle of 90 is straight up

    double angle = getAngle();
    double setpoint = inst.getTable(kNTArm).getEntry(kNTSetpoint).getDouble(kDefaultSetpoint);

    // The forward controll needed is proportional to the cosine of the angle
    double forward_power = fcp * Math.cos(Math.toRadians(angle));

    pid.setSetpoint(setpoint);
    double pid_power = pid.calculate(angle);

    double voltage = pid_power + forward_power;

    // Update the network table with the forward and PID power
    inst.getTable(kNTArm).getEntry(kForwardPower).setDouble(forward_power);
    inst.getTable(kNTArm).getEntry(kNTPidPower).setDouble(pid_power);
    inst.getTable(kNTArm).getEntry(kNTMotorSpeed).setDouble(voltage);

    armR.setVoltage(voltage);
    armL.setVoltage(voltage);
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
    inst.getTable(kNTArm).getEntry(kNTSetpoint).setDouble(point);
  }

  /**
   * @return The current angle of the arm
   */
  public void resetVoltage() {
    armL.setVoltage(kDefaultForwardParam * Math.cos(Math.toRadians(getAngle())));
    armR.setVoltage(kDefaultForwardParam * Math.cos(Math.toRadians(getAngle())));
  }

  public double getAngle() {
    // This is the only place where the arm encoder is read
    double angle = 360 * (armEncoder.getAbsolutePosition()) + kArmEncoderOffset;
    inst.getTable(kNTArm).getEntry(kNTCurrentAngle).setDouble(angle);
    return angle;
  }

  /**
   * @return The current setpoint of the PID controller
   */
  public double getSetpoint() {
    return pid.getSetpoint();
  }

  /**
   * Enable the arm for movement and set the motors to coast
   */
  public void enable(boolean setpoint_update) {
    if (setpoint_update) {
      setAngle(getAngle());
    }
    is_disabled = false;
    armR.setIdleMode(kEnabledMotorMode);
    armL.setIdleMode(kEnabledMotorMode);

    // Clear network table error code
    inst.getTable(kNTArm).getEntry(kNTErrorCode).setString("");
    inst.getTable(kNTArm).getEntry(kNTIsEnabled).setBoolean(true);

  }

  /**
   * Enable the arm for movement and set the motors to coast.
   * Setpoint is updated to prevent the sudden movement of the arm.
   */
  public void enable() {
    enable(true);
  }

  /**
   * Put arm in a safe state.
   */
  public void disable() {
    is_disabled = true;
    armR.setIdleMode(kDisabledMotorMode);
    armL.setIdleMode(kDisabledMotorMode);
    inst.getTable(kNTArm).getEntry(kNTIsEnabled).setBoolean(false);
  }

}