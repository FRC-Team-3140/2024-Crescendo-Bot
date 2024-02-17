package frc.robot.commands.L1Commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmToDistanceL1 extends Command {
    private final Arm arm = Arm.getInstance();
    private final double distance;
    // private double setpoint;
    private double angleTolerance = 1.0;

    /**
     *   This command sets the arm to a specific distance
     * 
     * @param arm
     * @param distance
     */
    public SetArmToDistanceL1(double distance) {
        this.distance = distance;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Code to initialize the command
        arm.setArmToShootDistance(distance);
    }

    @Override
    public void execute() {
        // Do nothing because the arm's PIDController will handle moving the arm to the target angle
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the arm is at the target angle
        return Math.abs( NetworkTableInstance.getDefault().getTable("kNTArm").getEntry("kNTSetpoint").getDouble(8)- arm.getAngle()) < angleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends
    }
}