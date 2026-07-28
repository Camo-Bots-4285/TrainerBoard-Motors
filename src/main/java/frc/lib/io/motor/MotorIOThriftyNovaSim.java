package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.thethriftybot.devices.ThriftyNova;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.motor.MotorIORev.RevFollower;
import frc.lib.io.motor.MotorIOThriftyNova.ThriftyNovaFollower;
import frc.lib.util.Device;
import frc.lib.util.PID;

/**
 * Generic simulation implementation of a motor controller.
 *
 * <p>
 * This class simulates the behavior of the motor controller only.
 * The mechanism physics are handled externally by WPILib simulations such as:
 * SingleJointedArmSim, ElevatorSim, etc.
 */
public class  MotorIOThriftyNovaSim implements MotorIOSim {
    //Variables
    //region 
    // Current simulated sensor values
    private Angle position = Rotation.zero();

    private AngularVelocity velocity = RotationsPerSecond.zero();

    private AngularAcceleration acceleration = RotationsPerSecondPerSecond.zero();

    // Controller output
    private Voltage appliedVoltage = Volts.zero();

    // Control state
    private ControlType controlMode = ControlType.DUTYCYCLE;

    private ControlType idleMode = ControlType.COAST;

    // PID controllers
    private PIDController positionPID;
    private double positionkf;

    private PIDController velocityPID;
    private double velocitykf;

    // Position profiling

    private Angle goalPosition = Rotation.of(Double.NaN);

    private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints constraints;

    private double lastGoal = Double.NaN;

    private double lastTime = Timer.getFPGATimestamp();

    // Velocity profiling

    private AngularVelocity goalVelocity = RotationsPerSecond.of(Double.NaN);

    private SlewRateLimiter velocityLimiter;

    private double maxVelocity = Double.POSITIVE_INFINITY;
//endregion

    /**
     * Creates a generic motor controller simulation.
     *
     * @param maxVelocity     Optional maximum velocity limit
     * @param maxAcceleration Optional maximum acceleration limit
     */
    public MotorIOThriftyNovaSim(
            ThriftyNova.ThriftyNovaConfig config,
            double RotorToSensorRatio,
            double SensorToMechanismRatio,
            DCMotor gearBox,
            Optional<AngularVelocity> maxVelocity,
            Optional<AngularAcceleration> maxAcceleration,
            ThriftyNovaFollower... followerData) {

                
        positionPID = config.pid0.pid;
        positionkf = config.pid0.f;
        velocityPID = config.pid1.pid;
        velocitykf = config.pid1.f;


        maxVelocity.ifPresent(
                value -> this.maxVelocity = value.in(RotationsPerSecond));

        double maxAccel = maxAcceleration
                .map(
                        value -> value.in(
                                RotationsPerSecondPerSecond))
                .orElse(
                        Double.POSITIVE_INFINITY);

        constraints = new TrapezoidProfile.Constraints(
                maxVelocity
                        .map(v -> v.in(RotationsPerSecond))
                        .orElse(Double.POSITIVE_INFINITY),

                maxAccel);

        if (Double.isFinite(maxAccel)) {

            velocityLimiter = new SlewRateLimiter(maxAccel);

        }

    }

    @Override
    public void updateInputs(MotorInputs inputs) {

        inputs.connected = true;

        inputs.position = position;

        inputs.velocity = velocity;

        inputs.appliedVoltage = appliedVoltage;

        /*
         * Generic simulation does not know current draw.
         * A mechanism simulation can override this if desired.
         */
        inputs.supplyCurrent = null;

        inputs.torqueCurrent = null;

        inputs.temperature = null;

        inputs.goalPosition = goalPosition;

        inputs.goalVelocity = goalVelocity;

        inputs.positionError = goalPosition.minus(position);

        inputs.velocityError = goalVelocity.minus(velocity);

        inputs.activeTrajectoryPosition = Rotations.of(
                profileState.position);

        inputs.activeTrajectoryVelocity = RotationsPerSecond.of(
                profileState.velocity);

        inputs.controlType = controlMode;

        // Same behavior as hardware IO
        controlMode = idleMode;

    }

    // -------------------------------------------------------
    // Sensor simulation input
    //region

    @Override
    public void setPosition(Angle position) {
        this.position = position;
    }

    @Override
    public void setRotorVelocity(
            AngularVelocity velocity) {
        this.velocity = velocity;
    }

    @Override
    public void setRotorAcceleration(
            AngularAcceleration acceleration) {
        this.acceleration = acceleration;
    }

    @Override
    public double getRotorToSensorRatio() {
        return 1.0;
    }
//endregion
    
    //--------------------------------------------------------
    // Contorl Types
    //region

    @Override
    public void runVoltage(
            Voltage voltage) {

        controlMode = ControlType.VOLTAGE;

        appliedVoltage = voltage;

    }

    @Override
    public void runCurrent(Current current) {
        // Not supported
    }

    @Override
    public void runCurrent(
            Current current,
            double dutyCycle) {
        // Not supported
    }

    @Override
    public void runDutyCycle(
            double dutyCycle,
            boolean ignoringSoftLimits) {

        controlMode = ControlType.DUTYCYCLE;

        appliedVoltage = Volts.of(
                dutyCycle * 12.0);

    }


    @Override
    public void runPosition(
            Angle position,
            PIDSlot slot) {

        controlMode = ControlType.POSITION;

        goalPosition = position;

        TrapezoidProfile profile = new TrapezoidProfile(
                constraints);

        double now = Timer.getFPGATimestamp();

        double dt = now - lastTime;

        lastTime = now;

        profileState = profile.calculate(
                dt,
                profileState,
                new TrapezoidProfile.State(
                        position.in(Rotations),
                        0));

        double output = positionPID.calculate(
                this.position.in(Rotations),
                profileState.position);

        appliedVoltage = Volts.of(
                MathUtil.clamp(
                        output,
                        -12,
                        12));

    }

    @Override
    public void runPosition(
            Angle position,
            PIDSlot slot,
            AngularVelocity cruiseVelocity,
            AngularAcceleration acceleration) {

        controlMode = ControlType.POSITION;

        goalPosition = position;

        double goal = position.in(Rotations);

        // Reset profile when the command changes
        if (goal != lastGoal) {

            profileState = new TrapezoidProfile.State(
                    this.position.in(Rotations),
                    this.velocity.in(RotationsPerSecond));

            lastGoal = goal;

        }

        TrapezoidProfile profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        cruiseVelocity.in(
                                RotationsPerSecond),

                        acceleration.in(
                                RotationsPerSecondPerSecond)));

        double now = Timer.getFPGATimestamp();

        double dt = now - lastTime;

        lastTime = now;

        profileState = profile.calculate(
                dt,
                profileState,
                new TrapezoidProfile.State(
                        goal,
                        0));

        double output = positionPID.calculate(
                this.position.in(Rotations),
                profileState.position);

        appliedVoltage = Volts.of(
                MathUtil.clamp(
                        output,
                        -12,
                        12));

    }

    @Override
    public void runUnprofiledPosition(
            Angle position,
            PIDSlot slot) {

        controlMode = ControlType.POSITION;

        goalPosition = position;

        double output = positionPID.calculate(
                this.position.in(Rotations),
                position.in(Rotations));

        appliedVoltage = Volts.of(
                MathUtil.clamp(
                        output,
                        -12,
                        12));

    }


    @Override
    public void runVelocity(
            AngularVelocity velocity,
            PIDSlot slot) {

        controlMode = ControlType.VELOCITY;

        goalVelocity = velocity;

        double output = velocityPID.calculate(
                this.velocity.in(
                        RotationsPerSecond),

                velocity.in(
                        RotationsPerSecond));

        appliedVoltage = Volts.of(
                MathUtil.clamp(
                        output,
                        -12,
                        12));

    }

    @Override
    public void runVelocity(
            AngularVelocity velocity,
            AngularAcceleration acceleration,
            PIDSlot slot) {

        controlMode = ControlType.VELOCITY;

        double target = MathUtil.clamp(
                velocity.in(
                        RotationsPerSecond),

                -maxVelocity,
                maxVelocity);

        if (velocityLimiter != null) {
            target = velocityLimiter.calculate(
                    target);
        }

        goalVelocity = RotationsPerSecond.of(target);

        double output = velocityPID.calculate(
                this.velocity.in(
                        RotationsPerSecond),

                target);

        appliedVoltage = Volts.of(
                MathUtil.clamp(
                        output,
                        -12,
                        12));

    }
//endregion
    
    // -------------------------------------------------------
    // Configuration
    //region

    @Override
    public void setPID(
            PIDSlot slot,
            PID pid) {

        positionPID.setPID(
                pid.P(),
                pid.I(),
                pid.D());

        velocityPID.setPID(
                pid.P(),
                pid.I(),
                pid.D());

    }

    @Override
    public void setEncoderPosition(
            Angle position) {

        this.position = position;

    }

    @Override
    public void setSupplyCurrentLimit(
            Current current) {

        // Not modeled in generic sim

    }

    //endregion

    // -------------------------------------------------------
    // Idle modes
    //region

    @Override
    public void runCoast() {

        idleMode = ControlType.COAST;

        appliedVoltage = Volts.zero();

    }

    @Override
    public void runBrake() {

        idleMode = ControlType.BRAKE;

        appliedVoltage = Volts.zero();

    }
    //region

    @Override
    public int getNumberOfMotors() {
        return 1;
    }

    @Override
    public void close() {
        // Nothing to close
    }

}
