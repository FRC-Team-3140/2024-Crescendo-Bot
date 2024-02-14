// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Made SpeakerShoot the X keybind on the xbox controller so we can test it

package frc.robot.commands.L3Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.L1Commands.SetArmToDistanceL1;
import frc.robot.commands.L1Commands.ShootSpeakerL1;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * This class represents a command for shooting at a specific distance using the speaker mechanism.
 * It extends the Command class and implements the Constants interface.
 */
public class SpeakerShootDistanceL3 extends Command implements Constants {

    public IntakeShooter intakeShooter; 
    public SwerveDrive swerve;
    // private InterpolatingDoubleTreeMap angleInterpolator;
    public Arm arm;

    public SpeakerShootDistanceL3() {
        intakeShooter = IntakeShooter.getInstance();

        // TODO: This is a good interpolator but it needs to be associated with the arm.  I am moving it to the arm subsystem so it is avalible for all commands. -DB
        // angleInterpolator = new InterpolatingDoubleTreeMap();/* Add your inverseInterpolator, interpolator, and comparator here */
        // angleInterpolator.put(52.0 * .0254,14.0 ); //14 Degrees and 42 inches measured to the inside of the bot perimiter
        // angleInterpolator.put(72.0 * .0254,24.0 );
        // angleInterpolator.put(89.0 * .0254,32.0 );
        // angleInterpolator.put(122.0 * .025,37.4);
        // angleInterpolator.put(129.0 * .0254,38.5);
        // angleInterpolator.put(148.375 * .0254,39.8);
        this.swerve = SwerveDrive.getInstance();
        this.arm = Arm.getInstance();
        
        
    }
    SequentialCommandGroup test;
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Use the InterpolatingTreeMap to get the interpolated voltage for the key (e.g., joystick position)
        // PLEASE IGNORE the joystick part, this will be connected to the camera but its not ready yet

        // TODO: distance and angle should be estimated from the camera subsystem -DB
        double distance = Math.hypot(swerve.getPose().getX(), swerve.getPose().getY()- (216*.0254));
        System.out.println("Distance From Shooter" + distance);
        NetworkTableInstance.getDefault().getTable("distance").getEntry("N").setDouble(distance);
        //double interpolatedAngle = angleInterpolator.get(distance);
        //NetworkTableInstance.getDefault().getTable("distance").getEntry("Angle").setDouble(interpolatedAngle);
        // Set the shooter voltage based on the interpolated value
        test = new SequentialCommandGroup(
            new SetArmToDistanceL1(distance), // TODO: Refactored. Test that this still works. -DB
            new ShootSpeakerL1(10,3)
        );
        test.schedule();
        
    }
    @Override
    public boolean isFinished() {
        return test.isFinished() || !test.isScheduled();
    }

    // Other methods for isFinished(), end(), etc., can be added if needed.
}
