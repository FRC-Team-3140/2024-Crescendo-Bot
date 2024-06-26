// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Arm class represents the subsystem responsible for controlling the arm mechanism of the robot.
 * It handles the PID control, motor configuration, setpoints, and network table communication.
 */
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
  private static final int kMotorCurrentLimit = 30; // The current limit for the arm motors
  private static final boolean kArmRightReversed = true; // Motor direction for right arm
  private static final boolean kArmLeftReversed = false; // Motor direction for left arm
  private static final IdleMode kEnabledMotorMode = IdleMode.kBrake; // Motor mode when enabled
  private static final IdleMode kDisabledMotorMode = IdleMode.kBrake; // Motor mode when disabled

  // Constants for the PID controller
  private static final double kDefaultP = .37; // Proportional gain
  private static final double kDefaultI = 0.02; // Integral gain
  private static final double kDefaultD = 0.015; // Derivative gain

  // Constants for the arm setpoint
  private static final double kDefaultSetpoint = 0.0; // The starting set point for the arm
  private static final double kMaxSetpoint = 94.0; // Maximum setpoint; Test again with Amp
  private static final double kMinSetpoint = 7.5; // Minimum setpoint

  // Favorite setpoints

  public static final double kSetpointShoot = 14.0; // The setpoint for shooting
  public static final double kSetpointIntakeDown = 7.5; // The setpoint for intaking
  public static final double kSetpointIntakeReady = 28.0; // The ready for intake but off the ground for movement and
                                                          // protection
  public static final double kSetpointAmp = 94.0; // The ready for intake but off the ground for movement and protection
  public static final double kSetpointMove = 65.0; // The ready for intake but off the ground for movement and
                                                   // protection

  public static final double kDistanceShootLinearTrim = 0.0; // The linear trim for the shooting distance
  public static final double kDistanceShootAdditiveTrim = 0.0; // The additive trim for the shooting distance

  // Constants for the arm control
  private static final double kDefaultForwardParam = .36; // The default forward control parameter
  private static final double kArmEncoderOffset = -155; // The offset of the arm encoder from the zero position //
                                                        // degrees
  private static final double maxAcceleration = 480;
  private static final double maxVelocity = 360;

  // Create a NetworkTable instance to enable the use of NetworkTables
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private boolean is_disabled = false; // Disables the arm if the encoder is not connected

  private CANSparkMax armR;
  private CANSparkMax armL;

  private ProfiledPIDController pid;

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
   * instances of the class from being created.
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
    armL.setSmartCurrentLimit(kMotorCurrentLimit);
    armL.burnFlash();

    // Create entries for P, I, and D values
    NetworkTableEntry pEntry = inst.getTable(kNTArm).getEntry(kNTP);
    NetworkTableEntry iEntry = inst.getTable(kNTArm).getEntry(kNTI);
    NetworkTableEntry dEntry = inst.getTable(kNTArm).getEntry(kNTD);
    // NetworkTableEntry setpointEntry =
    // inst.getTable(kNTArm).getEntry(kNTSetpoint);
    NetworkTableEntry fcpEntry = inst.getTable(kNTArm).getEntry(kNTForwardParam);
    inst.getTable(kNTArm).getEntry(kNTP).setDouble(kDefaultP);
    inst.getTable(kNTArm).getEntry(kNTI).setDouble(kDefaultI);
    inst.getTable(kNTArm).getEntry(kNTD).setDouble(kDefaultD);

    // Set the entries to be persistent
    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();
    // setpointEntry.setPersistent();
    fcpEntry.setPersistent();

    double p = inst.getTable(kNTArm).getEntry(kNTP).getDouble(kDefaultP);
    double i = inst.getTable(kNTArm).getEntry(kNTI).getDouble(kDefaultI);
    double d = inst.getTable(kNTArm).getEntry(kNTD).getDouble(kDefaultD);
    double setpoint = inst.getTable(kNTArm).getEntry(kNTSetpoint).getDouble(kDefaultSetpoint);
    fcp = inst.getTable(kNTArm).getEntry(kNTForwardParam).getDouble(kDefaultForwardParam);

    pid = new ProfiledPIDController(p, i, d, new Constraints(maxVelocity, maxAcceleration));
    pid.setGoal(setpoint);
    // pid.setTolerance(.1);
    pid.setIntegratorRange(-.125, .25);

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
    encoderConnected();

    angleInterpolator = new InterpolatingDoubleTreeMap();// Add your inverseInterpolator, interp2lator, and comparator
                                                         // here

    angleInterpolator.put(1.3, 16.0);
    angleInterpolator.put(2.067, 22.0);
    angleInterpolator.put(2.923, 30.0);
    angleInterpolator.put(3.287, 34.0);
    angleInterpolator.put(4.266, 38.0);

    // Set the arm to disabled by default.
    disable();
  }

  /**
   * Checks if the ArmEncoder is connected.
   * If the ArmEncoder is not connected, it sets an error message and disables the arm.
   */
  private void encoderConnected() {
    // check that ArmEncoder is connected
    if (!armEncoder.isConnected()) {
      inst.getTable(kNTArm).getEntry(kNTErrorCode).setString("Arm encoder not connected");
      is_disabled = true;
    }
  }


  /**
   * This method is called periodically to update the arm subsystem.
   * It ensures that the PID values are updated from the network table,
   * retrieves the current setpoint from the network table, checks the limits,
   * and updates the PID controller with the current setpoint. Finally, it
   * updates the power of the arm motor based on the calculated PID output.
   */
  @Override
  public void periodic() {
    // make sure pid values are updated from the network table
    pid.setP(inst.getTable(kNTArm).getEntry(kNTP).getDouble(kDefaultP));
    pid.setI(inst.getTable(kNTArm).getEntry(kNTI).getDouble(kDefaultI));
    pid.setD(inst.getTable(kNTArm).getEntry(kNTD).getDouble(kDefaultD));

    // Get the current setpoint in the network table. Check the limits and then
    // update the PID controller with the current setpoint
    double setpoint = inst.getTable(kNTArm).getEntry(kNTSetpoint).getDouble(kDefaultSetpoint);

    if (setpoint > kMaxSetpoint) {
      setpoint = kMaxSetpoint;
    } else if (setpoint < kMinSetpoint) {
      setpoint = kMinSetpoint;
    }

    // Update the setpoint incase it has changed

    // set the arm motor power
    updatePower(pid.calculate(getAngle()));

  }

  // TODO: I think this function is wrong and needs to be removed. It seems to be
  // a dublicate of setAngle.
  /**
   * Sets the arm to the specified angle.
   * 
   * @param setPoint the desired angle to set the arm to
   */
  public void setArmToAngle(double setPoint) {
    double angle = getAngle();
    double setpoint = setPoint;

    // The forward controll needed is proportional to the cosine of the angle
    double forward_power = kDefaultForwardParam
        * Math.cos(Math.toRadians(angle));

    pid.setGoal(setpoint);
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
    // setArmToAngle(interpolatedAngle);
    // double interpolatedAngle = angleInterpolator.get(distance);

    // double interpolatedAngle = Math.max(16, -130.725 *
    // Math.exp(distance*-1.07775) + 43.0501);
    setArmToAngle(interpolatedAngle);
    return -149.003 * Math.max(16, -132.744 * Math.exp(distance * -1.06174) + 45.2311); // TODO: BAD! Fix this.
  }

  /**
   * Estimates the angle for a given distance.
   * 
   * @param distance the distance for which to estimate the angle
   * @return the estimated angle
   */
  public double estimateAngleForDistance(double distance) {
    double interpolatedAngle = angleInterpolator.get(distance);

    // The trim for the shooting distance makes small adjustments to the angle
    return (1.0 + kDistanceShootLinearTrim) * interpolatedAngle + kDistanceShootAdditiveTrim;
  }

  /**
   * Sets the power of the arm motors
   * 
   * @param power
   */
  private void updatePower(double power) {
    // check that the arm is not disabled
    if (is_disabled || !armEncoder.isConnected()) {

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
    double voltage = forward_power;
    pid.setGoal(setpoint);
    double pid_power = pid.calculate(angle);
    // if(Math.abs(angle - setpoint) < 3 && setpoint - angle < 0 ){
    // voltage += Math.signum(setpoint-angle) * .5;
    // voltage = pid_power + forward_power;
    // }else{
    voltage += pid_power;
    // }

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

  /**
   * Returns the current angle of the arm.
   * This method reads the arm encoder and calculates the angle based on the encoder value.
   * The calculated angle is then stored in the network table for monitoring purposes.
   * 
   * @return The current angle of the arm in degrees.
   */
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
    return pid.getGoal().position;
  }

  /**
   * Enable the arm for movement and set the motors to coast.
   * 
   * @param setpoint_update true if the setpoint should be updated, false otherwise
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