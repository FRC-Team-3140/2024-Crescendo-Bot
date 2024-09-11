// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPathplannerPath;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Arm;

public class CameraTest extends SequentialCommandGroup {

  /** Creates a new CameraTest. */
  public CameraTest() {
    pickupNote intake = new pickupNote(false, RobotContainer.swerve, Camera.getInstance());
    Command back = new FollowPathplannerPath("To Speaker", RobotContainer.swerve);
    Command out = new FollowPathplannerPath("Straight Line", RobotContainer.swerve);
    Command arm = new SetArmToAngleL1(Arm.kSetpointShoot);
    Command shoot = new ShootSpeakerL1(Constants.shooterVoltage, Constants.intakeVoltage);

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(out, intake, back, arm, shoot);
  }
}
