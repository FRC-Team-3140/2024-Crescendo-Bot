// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Intake and Shooter related
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Color sensor related
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class IntakeShooter extends SubsystemBase {

    public  static IntakeShooter instance = null;
    public CANSparkMax intakeMotor = new CANSparkMax(20, MotorType.kBrushless);
    public CANSparkMax shooterA = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax shooterB = new CANSparkMax(8, MotorType.kBrushless);
    public RelativeEncoder encoderA = shooterA.getEncoder();
    public RelativeEncoder encoderB = shooterB.getEncoder();

// Change the I2C port below to match the connection of the color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

// This object is constructed with an I2C port as a parameter
// This device will be automatically initialized with default parameters
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

// ColorMatch object is used to register and detect known colors
// This object uses a simple euclidian distance to estimate the closest match with the given confidence range
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kOrangeTarget = new Color(255, 85, 0);

// Returns the instance
    public static synchronized IntakeShooter getInstance() {
        if (instance == null) {
            instance = new IntakeShooter();
        }
        return instance;
    }
// Sets the motor to neutral
    public IntakeShooter() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        m_colorMatcher.addColorMatch(kOrangeTarget);
    }

// Sets the speed of the intake motor through power
    public void intake(double power) {
        intakeMotor.set(power);
    }

//Sets the speed of the shooter's motor, make sure one is negative and one is postive
    public void shoot (double power) {
        shooterA.set(-power);
        shooterB.set(power);
    } 

    @Override
    public void periodic() {
    
    // The method GetColor() returns a normalized color value from the sensor
    // To read the raw color, use GetRawColor()
    Color detectedColor = m_colorSensor.getColor();

    // Run the color match algorithm on our detected color
    String colorString;
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kOrangeTarget) {
      colorString = "Orange";
    } else {
      colorString = "Unknown";
    }

    System.out.println(colorString);

    //Open Smart Dashboard to see the color detected by the sensor.
    SmartDashboard.putNumber("Orange", detectedColor.orange);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    }

}