package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CameraLeftTwoNote extends SequentialCommandGroup{
    public CameraLeftTwoNote(){
        super(AutoBuilder.buildAuto("CameraLeftTwoNote1") );
    }
}
