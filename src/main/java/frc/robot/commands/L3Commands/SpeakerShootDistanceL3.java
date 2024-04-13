// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made SpeakerShoot the X keybind on the xbox controller so we can test it

package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;

/**
 * This class represents a command for shooting at a specific distance using the
 * speaker mechanism.
 * It extends the Command class and implements the Constants interface.
 */
public class SpeakerShootDistanceL3 extends SequentialCommandGroup {

    public boolean finished;

    /**
     * A command that shoots the speaker at a specific distance level in Level 3.
     */
    public SpeakerShootDistanceL3() {

        super(
                new ParallelCommandGroup(
                        new SetArmToDistanceL1(), // TODO: Refactored. Test that this still works. -DB
                        new ShootSpeakerL1(10, 5),
                        new PrintCommand("Command ended")));
    }
}
