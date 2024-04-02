package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.commands.L1Commands.ShootSpeakerOverrideL1;
import frc.robot.commands.L3Commands.CameraShootDistanceL3;

public class LeftThreeNote extends SequentialCommandGroup{
    static SequentialCommandGroup speakerShoot = new SequentialCommandGroup(new SetArmToDistanceL1(), new ParallelRaceGroup( new ShootSpeakerL1(10, 5), new WaitCommand(2.5)),new ParallelRaceGroup(new ShootSpeakerOverrideL1(10, 3), new WaitCommand(.3)));

    public LeftThreeNote(){
        super(AutoBuilder.buildAuto("LeftThreeNote1"), new CameraShootDistanceL3(), speakerShoot, AutoBuilder.buildAuto("LeftThreeNote2"));
    }
}
