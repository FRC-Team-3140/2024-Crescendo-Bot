package frc.robot.commands.L2Commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualIntakeControlL2 extends Command {
    private final Arm arm;
    private final Intake intake;
    private final DoubleSupplier intakeSpeedSupplier;
    private final BooleanSupplier armPositionSupplier;

    public ManualIntakeControlL2(DoubleSupplier intakeSpeedSupplier, BooleanSupplier armPositionSupplier) {
        this.arm = Arm.getInstance();
        this.intake = Intake.getInstance();
        this.intakeSpeedSupplier = intakeSpeedSupplier;
        this.armPositionSupplier = armPositionSupplier;

        // Declare subsystem dependencies
        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        // Initialization code, if any
    }

    @Override
    public void execute() {
        // Control the intake based on the float supplier
        double intakeSpeed = intakeSpeedSupplier.getAsDouble();
        boolean arm_position = armPositionSupplier.getAsBoolean();

        if (intakeSpeed > 0.1) {
            // Linear interpret
            // 0.1 is intake ready
            // 0.9 is intake down
            // map from 0.0 to 1.0
            double arm_angle = (intakeSpeed - 0.1) / 0.8;
            if(arm_angle > 1.0) arm_angle=1.0;
            if(arm_angle < 0.0) arm_angle=0.0;

            arm_angle = (1.0-arm_angle)*Arm.kSetpointIntakeReady + arm_angle*Arm.kSetpointIntakeDown;
            arm.setAngle(arm_angle);
        } else {
            arm.setAngle(Arm.kSetpointIntakeReady);
        }

        if (intakeSpeed > 0.5) {
            intake.setIntakeVoltage(5.0);
        } else {
            intake.setIntakeVoltage(0.0);
        }

        // Control the arm based on the boolean supplier
        if (armPositionSupplier.getAsBoolean()) {
            arm.setAngle(Arm.kSetpointMove);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake and arm when the command ends
        intake.setIntakeVoltage(0);
        arm.setAngle(arm.getAngle());
    }

    @Override
    public boolean isFinished() {
        // This command runs until interrupted
        return false;
    }
}
