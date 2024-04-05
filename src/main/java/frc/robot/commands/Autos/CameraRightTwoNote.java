package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.pickupNote;
import frc.robot.commands.L1Commands.SetArmToAngleL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.SwerveDrive;

public class CameraRightTwoNote extends SequentialCommandGroup{
    static pickupNote intake = new pickupNote(false, SwerveDrive.getInstance(), Camera.getInstance());
    // static CameraShootDistanceL3 shoot = new CameraShootDistanceL3();
    static ParallelRaceGroup shootSpeaker = new ParallelRaceGroup(new WaitCommand(3), new ShootSpeakerL1(6.5, 5));
    public CameraRightTwoNote(){
        super(new SetArmToAngleL1(16), shootSpeaker, AutoBuilder.buildAuto("CameraRightTwoNote1"), new WaitCommand(5), new pickupNote(false, RobotContainer.swerve,
      RobotContainer.camera) ,  AutoBuilder.buildAuto("CameraRightTwoNote2"));
    }
}
