package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;

/**
 * Represents a command group for executing a specific autonomous routine called "LeftThreeNote".
 * This command group consists of three sequential commands: 
 * 1. An auto routine named "LeftThreeNote1" built using AutoBuilder.
 * 2. A sequential command group named "speakerShoot" which includes:
 *    - Setting the arm to a specific distance using SetArmToDistanceL1 command.
 *    - Shooting the speaker with a power of 10 and a speed of 5 using ShootSpeakerL1 command, with a timeout of 2.5 seconds.
 * 3. An auto routine named "LeftThreeNote2" built using AutoBuilder.
 */
public class LeftThreeNote extends SequentialCommandGroup {
    static SequentialCommandGroup speakerShoot = new SequentialCommandGroup(new SetArmToDistanceL1(),
            new ShootSpeakerL1(10, 5).withTimeout(2.5));

    public LeftThreeNote() {
        super(AutoBuilder.buildAuto("LeftThreeNote1"), speakerShoot, AutoBuilder.buildAuto("LeftThreeNote2"));
    }
}
