package frc.robot.commands.L2Commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualIntakeControlL2 extends Command {
    private final Arm arm;
    private final Intake intake;
    private Shooter shooter;

    private final DoubleSupplier intakeSupplier;
    private final BooleanSupplier ampShootSupplier;
    private BooleanSupplier defaultShootSupplier;

    public ManualIntakeControlL2(DoubleSupplier intakeSupplier, BooleanSupplier defaultShootSupplier,
            BooleanSupplier ampShootSupplier) {

        this.arm = Arm.getInstance();
        this.shooter = Shooter.getInstance();
        this.intake = Intake.getInstance();

        this.intakeSupplier = intakeSupplier;
        this.defaultShootSupplier = defaultShootSupplier;
        this.ampShootSupplier = ampShootSupplier;

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
        double intakeValue = intakeSupplier.getAsDouble();
        boolean ampShoot = ampShootSupplier.getAsBoolean();
        boolean defaultShoot = defaultShootSupplier.getAsBoolean();

        // priority: defaultShoot > ampShoot > intakeSpeed
        if (defaultShoot) {
            arm.setAngle(Arm.kSetpointShoot);
            shooter.set(Shooter.kShootSpeed);

            double speed = shooter.getShooterSpeed();
            System.out.printf("Shooter Speed: %f\n",speed);
            if(arm.isAtSetpoint() && (speed > 4000.0)){ // launch
                intake.setIntakeVoltage(Intake.kSetpointShoot);
            } else {
                intake.setIntakeVoltage(Intake.kSetpointStop);
            }

        } else if (ampShoot) {
            arm.setAngle(Arm.kSetpointAmp);
            shooter.set(Shooter.kAmpSpeed);

            double speed = shooter.getShooterSpeed();
            System.out.printf("Shooter Speed: %f\n",speed);
            if((speed > 1000.0)){ // launch
                intake.setIntakeVoltage(Intake.kSetpointShoot);
            } else {
                intake.setIntakeVoltage(Intake.kSetpointStop);
            }


        } else {
            if (intakeValue < 0.1) {
                
                if (arm.getSetpoint() < Arm.kSetpointIntakeReady) {
                    arm.setAngle(Arm.kSetpointIntakeReady);
                }
                if( arm.getSetpoint() > Arm.kSetpointMove){
                    arm.setAngle(Arm.kSetpointMove);
                }

                intake.setIntakeVoltage(Intake.kSetpointStop);
                shooter.set(Shooter.kStopSpeed);
            } else {
                // Pickup the note

                shooter.set(0);

                if(intake.hasNote()){
                    intake.setIntakeVoltage(0.0);
                } else {
                    intake.setIntakeVoltage(5.0);
                }
                // Linear interpret
                // 0.1 is intake ready
                // 0.9 is intake down
                // map from 0.0 to 1.0
                double arm_angle = (intakeValue - 0.1) / 0.8;
                if (arm_angle > 1.0)
                    arm_angle = 1.0;
                if (arm_angle < 0.0)
                    arm_angle = 0.0;

                arm_angle = (1.0 - arm_angle) * Arm.kSetpointIntakeReady + arm_angle * Arm.kSetpointIntakeDown;
                arm.setAngle(arm_angle);

            }
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
