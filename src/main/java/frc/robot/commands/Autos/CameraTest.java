// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pickupNote;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

public class CameraTest extends SequentialCommandGroup {
  static pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
  static Command out = AutoBuilder.buildAuto("Opposite of Straight Line");
  static Command back = AutoBuilder.buildAuto("Straight Line");

  /** Creates a new CameraTest. */
  public CameraTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    super(out, intake, back);
  }
}