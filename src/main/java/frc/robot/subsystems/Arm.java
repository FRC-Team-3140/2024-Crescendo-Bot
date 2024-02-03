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

    // Set the entries to be persistent
    pEntry.setPersistent();
    iEntry.setPersistent();
    dEntry.setPersistent();


    double p = inst.getTable("Arm").getEntry("P: ").getDouble(kDefaultP);
    double i = inst.getTable("Arm").getEntry("I: ").getDouble(kDefaultI);
    double d = inst.getTable("Arm").getEntry("D: ").getDouble(kDefaultD);

    pid = new PIDController(p, i, d);

    armEncoder = new DutyCycleEncoder(kArmEncoderID);
  }

  @Override
  public void periodic() {
    pid.setP(inst.getTable("Arm").getEntry("P: ").getDouble(0));
    pid.setI(inst.getTable("Arm").getEntry("I: ").getDouble(0));
    pid.setD(inst.getTable("Arm").getEntry("D: ").getDouble(0));
  }


  public void setAngle(double point) {
    // Was setPoit : LOL ;)

    pid.setSetpoint(point);
  }

  public double getAngle() {
    return armEncoder.getAbsolutePosition();
  }
}
