package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.Device;


/**
 * Simulated implementation of {@link MotorIORev} for REV Robotics motors using WPILib simulation.
 * Implements {@link MotorIOSim} to provide simulation-specific behavior.
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

    // Note: motor and controller are inherited from MotorIORev. Do not redeclare them here --
    // a subclass field with the same name hides the parent's rather than replacing it, so the two
    // drift apart silently.

    private final SparkSim simState;

    // Seeded with the current time, not zero. RobotController.getMeasureTime() is time since the
    // program started, so starting from zero would make the first simulation timestep as long as
    // the robot code has been running and launch the mechanism to its limit on the first loop.
    private Time lastTime = RobotController.getMeasureTime();

    private final double rotorToSensorRatio;
    private final double sensorToMechanismRatio;
    private AngularVelocity velocity = RotationsPerSecond.zero();

    /**
     * Constructs a MotorIORevSim instance.
     *
     * @param name Name of the motor, used for logging and dashboard alerts
     * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios.
     *  Note: pass though a SparkFlex or SparkMax config or else max motion will not behave as expected
     * @param CAN CAN device reference containing the motor's CAN ID
     * @param rotorToSensorRatio Rotor rotations per sensor rotation
     * @param sensorToMechanismRatio Sensor rotations per mechanism rotation
     * @param gearBox DCMotor gearbox model used by the simulation
     * @param followerData Configuration data for the follower motor(s), can be empty if no
     *        followers
     *
     * @see MotorIO
     */
    public MotorIORevSim(
        String name,
        SparkBaseConfig config,
        Device.CAN CAN,
        double rotorToSensorRatio,
        double sensorToMechanismRatio,
        DCMotor gearBox,
        RevFollower... followerData)
    {
        super(name, config, CAN, followerData);

        this.rotorToSensorRatio = rotorToSensorRatio;
        this.sensorToMechanismRatio = sensorToMechanismRatio;

        if (config instanceof SparkFlexConfig) {
            simState = new SparkFlexSim((SparkFlex) motor, gearBox);
        } else {
            simState = new SparkMaxSim((SparkMax) motor, gearBox);
        }

        // The superclass constructor already applied this config with kPersistParameters.
        // Applying it again would write to the SPARK's flash a second time on every boot.

        motor.createSimFaultManager();

        simState.enable();

    }

    @Override
    public void setRotorVelocity(AngularVelocity velocity) {
        this.velocity = velocity;
    }

    /**
     * Pushes a mechanism position from a WPILib physics simulation into the simulated motor.
     *
     * <p>Without this, the SPARK simulation only ever integrates its own velocity and never learns
     * the position the physics model computed, so it drifts away from the mechanism's real limits
     * and gravity behavior.
     *
     * <p>No gear ratio conversion is applied: {@code SparkSim.setPosition()} works in
     * post-conversion units, which the encoder conversion factors have already scaled to mechanism
     * rotations.
     */
    @Override
    public void setPosition(Angle position) {
        simState.setPosition(position.in(Rotations));
    }

    /**
     * Not supported. {@link SparkSim} models the motor from velocity alone and has no acceleration
     * input, so there is nothing to forward this to. Overridden explicitly so it is clear this is
     * deliberate rather than a missing implementation.
     */
    @Override
    public void setRotorAcceleration(AngularAcceleration acceleration) {
        // Intentionally empty -- see javadoc.
    }

    @Override
    public double getRotorToSensorRatio()
    {
        return rotorToSensorRatio;
    }

    @Override
    public double getSensorToMechanismRatio()
    {
        return sensorToMechanismRatio;
    }

    @Override
    public void updateInputs(MotorInputs inputs) {

        simState.setBusVoltage(RobotController.getBatteryVoltage());

        double deltaTime = RobotController.getMeasureTime().minus(lastTime).in(Seconds);
    
        simState.iterate(velocity.in(RotationsPerSecond), simState.getBusVoltage(), deltaTime);

        lastTime = RobotController.getMeasureTime();

        inputs.connected = true;

        inputs.position = Rotations.of(simState.getPosition());

        inputs.velocity = RotationsPerSecond.of(simState.getVelocity());

        inputs.appliedVoltage = Volts.of(simState.getAppliedOutput() * simState.getBusVoltage());

        inputs.supplyCurrent = Amps.of(simState.getMotorCurrent());

        // Signals the REV simulation cannot provide. These are logged as NaN rather than null:
        // AdvantageKit silently drops null fields when writing logs and throws when reading them
        // back during replay.
        inputs.torqueCurrent = Amps.of(UNSUPPORTED_SIGNAL);

        inputs.temperature = Celsius.of(UNSUPPORTED_SIGNAL);

        inputs.activeTrajectoryPosition = Rotations.of(UNSUPPORTED_SIGNAL);

        inputs.activeTrajectoryVelocity = RotationsPerSecond.of(UNSUPPORTED_SIGNAL);

        inputs.controlType = getCurrentControlType();

        // getSetpoint() returns whichever setpoint was last sent, in whatever units that control
        // mode uses. It is not tagged with a mode, so it is only meaningful once the active mode
        // says how to read it. Assigning it to both goals at once made one of them nonsense.
        inputs.goalPosition = Rotations.zero();
        inputs.goalVelocity = RotationsPerSecond.zero();
        inputs.positionError = Rotations.zero();
        inputs.velocityError = RotationsPerSecond.zero();

        if (isRunningPositionControl()) {
            inputs.goalPosition = Rotations.of(simState.getSetpoint());
            inputs.positionError = inputs.goalPosition.minus(inputs.position);
        } else if (isRunningVelocityControl()) {
            inputs.goalVelocity = RotationsPerSecond.of(simState.getSetpoint());
            inputs.velocityError = inputs.goalVelocity.minus(inputs.velocity);
        }

    }

    /**
     * Seeds the simulated encoder to a known mechanism position.
     *
     * <p>{@code SparkSim.setPosition()} works in post-conversion units, which the encoder
     * conversion factors have already scaled to mechanism rotations. Multiplying by the gear ratio
     * here would make the simulator disagree with a real robot by exactly that ratio.
     */
    @Override
    public void setEncoderPosition(Angle position) {
        simState.setPosition(position.in(Rotations));
    }

}
