package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

    public SparkBase motor;
    public SparkClosedLoopController controller;
    private SparkSim simState;
    private Time lastTime = Seconds.zero();

    private double RotorToSensorRatio;
    private double SensorToMechanismRatio;
    private AngularVelocity velocity = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);

    /**
     * Constructs a MotorIORevSim instance.
     * 
     * @param name Name of the motor
     * @param CAN CAN device reference containing the motor's CAN ID
     * @param isFlex True if using SparkFlex, false for SparkMax
     * @param gearBox DCMotor gearbox model
     * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios. 
     *  Note: pass though a SparkFlex or SparkMax config or else max motion will not behave as expected
     * @param followerData Configuration data for the follower motor(s), can be empty if no
     *        followers
     * 
     * @see MotorIO
     */
    public MotorIORevSim(
        String name,
        SparkBaseConfig config,
        Device.CAN CAN,
        double RotorToSensorRatio,
        double SensorToMechanismRatio,
        DCMotor gearBox,
        RevFollower... followerData)
    {
        super(name, config, CAN, followerData);

        motor = super.motor;
        this.RotorToSensorRatio = RotorToSensorRatio;
        this.SensorToMechanismRatio = SensorToMechanismRatio;

        if (config instanceof SparkFlexConfig) {
            simState = new SparkFlexSim((SparkFlex) motor, gearBox);
        } else {
            simState = new SparkMaxSim((SparkMax) motor, gearBox);
        }

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.createSimFaultManager();

        simState.enable();

    }

    @Override
    public void setRotorVelocity(AngularVelocity velocity) {
        this.velocity = velocity;
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
    public void updateInputs(MotorInputs inputs) {

        simState.setBusVoltage(RobotController.getBatteryVoltage());

        double deltaTime = RobotController.getMeasureTime().minus(lastTime).in(Seconds);
    
        simState.iterate(velocity.in(RotationsPerSecond), simState.getBusVoltage(), deltaTime);

        lastTime = RobotController.getMeasureTime();

        inputs.connected = true;

        inputs.position = Rotation.of(simState.getPosition());

        inputs.velocity = RotationsPerSecond.of(simState.getVelocity());

        inputs.appliedVoltage = Volts.of(simState.getAppliedOutput() * simState.getBusVoltage());

        inputs.supplyCurrent = Amps.of(simState.getMotorCurrent());

        inputs.torqueCurrent = null;

        inputs.temperature = null;

        inputs.activeTrajectoryPosition = null;

        inputs.activeTrajectoryVelocity = null;

        inputs.goalPosition = Rotation.of(simState.getSetpoint());

        inputs.goalVelocity = RotationsPerSecond.of(simState.getSetpoint());

        inputs.positionError = inputs.goalPosition.minus(inputs.position);

        inputs.velocityError =  inputs.goalVelocity.minus(inputs.velocity);

        inputs.controlType = null;

    }

    @Override
    public void close()
    {
        motor.close();
    }

    @Override
    public void setEncoderPosition (Angle postion){
        simState.setPosition(postion.times(RotorToSensorRatio*SensorToMechanismRatio).in(Rotation));
    }

}
