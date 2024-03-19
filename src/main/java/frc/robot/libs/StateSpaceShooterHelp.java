package frc.robot.libs;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class StateSpaceShooterHelp {
    private static final double spinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

    private static final double flywheelMomentOfInertia = 0.012958; // kg * m^2
    private static final double flywheelGearing = 1;
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(2), flywheelMomentOfInertia, flywheelGearing);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer;

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller;
        
    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop;
    
    public StateSpaceShooterHelp(){
        m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);

        m_controller = new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(12.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(1.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops.
        // The plant holds a state-space model of our flywheel. This system has the following properties:
        //
        // States: [velocity], in radians per second.
        // Inputs (what we can "put in"): [voltage], in volts.
        // Outputs (what we can measure): [velocity], in radians per second.
    
        m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);
    }
    
    public void reset(Vector<N1> startVelocity){
        m_loop.reset(startVelocity);
    }
    
    public void correct(Vector<N1> currentVelocity){
        m_loop.correct(currentVelocity);
    }

    public void predict(){
        m_loop.predict(.02); //Update time is constant
    }

    public void setSpeed(double speed){
        m_loop.setNextR(speed);
    }

    public double getVoltage(){
        return m_loop.getU(0);
    }

}
