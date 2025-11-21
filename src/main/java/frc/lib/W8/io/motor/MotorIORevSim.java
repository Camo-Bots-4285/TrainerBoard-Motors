package frc.lib.W8.io.motor;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.W8.util.Device;


/**
 * Simulated implementation of {@link MotorIORev} for REV Robotics motors using WPILib simulation.
 * Implements {@link MotorIOSim} to provide simulation-specific behavior.
 *
 * <p>
 * Constructor arguments:
 * <ul>
 * <li><b>name</b> - Name of the motor</li>
 * <li><b>id</b> - CAN ID of the motor</li>
 * <li><b>isFlex</b> - True if using SparkFlex, false for SparkMax</li>
 * <li><b>gearBox</b> - DCMotor gearbox model</li>
 * <li><b>config</b> - Motor configuration</li>
 * <li><b>followerData</b> - Varargs of follower motor data (ID and inversion)</li>
 * </ul>
 *
 * <p>
 * This class wraps a simulated SparkFlex or SparkMax motor, allowing position and velocity control
 * in a simulation environment. It provides methods to run the motor in position or velocity mode,
 * update simulation inputs, and manage simulation state.
 *
 * @see MotorIORev
 * @see MotorIOSim
 */
public class MotorIORevSim extends MotorIORev implements MotorIOSim {

    public SparkBase motor;
    public SparkClosedLoopController controller;
    private SparkSim simState;
    private Time lastTime = Seconds.zero();

    private double RotorToSensorRatio;
    private double SensorToMechanismRatio;

    /**
     * Constructs a MotorIORevSim instance.
     * 
     * @param name Name of the motor
     * @param id CAN ID of the motor
     * @param isFlex True if using SparkFlex, false for SparkMax
     * @param gearBox DCMotor gearbox model
     * @param config Motor configuration
     * @param followerData Varargs of follower motor data (ID and inversion)
     * 
     * @see MotorIO
     */
    public MotorIORevSim(
        String name,
        Device.CAN id,
        boolean isFlex,
        double RotorToSensorRatio,
        double SensorToMechanismRatio,
        DCMotor gearBox,
        SparkBaseConfig config,
        RevFollowerFollower... followerData)
    {
        super(name, id, isFlex, config,followerData);

        motor = this.getMotor();
        this.RotorToSensorRatio = RotorToSensorRatio;
        this.SensorToMechanismRatio = SensorToMechanismRatio;

        if (isFlex) {
            simState = new SparkFlexSim((SparkFlex) motor, gearBox);
        } else {
            simState = new SparkMaxSim((SparkMax) motor, gearBox);
        }

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        simState.enable();
    }

    @Override
    public void setPosition(Angle position)
    {
        simState.setPosition(position.in(Rotations));

    }


    @Override
    public void setRotorVelocity(AngularVelocity velocity)
    {
        simState.setVelocity(velocity.in(RotationsPerSecond));

    }

    @Override
    public double getRotorToSensorRatio()
    {
        return RotorToSensorRatio;
    }

    public double getSensorToMechanismRatio()
    {
        return SensorToMechanismRatio;
    }


    @Override
    public void updateInputs(MotorInputs inputs)
    {
        Time currentTime = Seconds.of(Timer.getTimestamp());
        double deltaTime = currentTime.minus(lastTime).in(Seconds); 

        simState.setBusVoltage(RoboRioSim.getVInVoltage());

        simState.iterate(
            simState.getVelocity(),
            simState.getBusVoltage(),
            deltaTime);

        inputs.position = Rotation.of(encoder.getPosition());
        inputs.velocity = RotationsPerSecond.of(encoder.getVelocity());
        inputs.appliedVoltage = Volts.of(simState.getAppliedOutput() * simState.getBusVoltage());
        inputs.supplyCurrent = Amps.of(simState.getMotorCurrent());
        inputs.temperature = Celsius.of(motor.getMotorTemperature());

        lastTime = currentTime;
    }

    @Override
    public void close()
    {
        super.motor.close();
    }

}
