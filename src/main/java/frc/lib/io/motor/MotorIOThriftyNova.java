package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig.PIDConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Device;
import frc.lib.util.PID;

public class MotorIOThriftyNova implements MotorIO {

    // Replace this with the actual ThriftyNova motor class
    protected final ThriftyNova motor;
    protected final ThriftyNova[] followers;
    protected final ThriftyNova.ThriftyNovaConfig config;



    //Variables for Postion Profile
    private Angle goalPosition = Rotation.of(Double.NaN);
    private TrapezoidProfile.State profileState;
    private TrapezoidProfile.Constraints kDefaultConstraints;
    private double lastGoal = Double.NaN;
    private double lastTime = Timer.getFPGATimestamp();

    //Variables for Velocity Profile
    private AngularVelocity goalVelocity = RotationsPerSecond.of(Double.NaN);
    private double kMaxVelocityRps;
    private SlewRateLimiter velocityLimiter;
    private double lastAccelLimit;

    //Control Type Tracker
    private ControlType idleMode;
    private boolean runningMotionProfile = false;
    private ControlType controlMode = ControlType.DUTYCYCLE;

    /**
     * Configuration data for a ThiftyNova motor that follows another ThiftyNova motor.
     * 
     * <p>
     * Follower motors mirror the output of a main motor, useful for mechanisms that require
     * multiple motors working together (like a dual-motor elevator).
     * 
     * @param id The CAN device ID of the follower motor
     * @param opposesMain Whether this follower should spin opposite to the main motor
     */
    public record ThriftyNovaFollower(Device.CAN id, boolean opposesMain) {}

    public MotorIOThriftyNova(
            String name,
            ThriftyNova.ThriftyNovaConfig config,
            Device.CAN main,
            Optional<AngularVelocity> maxVelocity,
            Optional<AngularAcceleration> maxAcceleration,
            Optional<Double> conversionFactor,
            ThriftyNovaFollower... followerData) {

            motor = new ThriftyNova(main.id());

            tryUntilOk(
                motor,
                5,
                () -> motor.applyConfig(config)
            );

            followers = new ThriftyNova[followerData.length];

            //For loop that iterates for each follower mototr
            for (int i = 0; i < followerData.length; i++) {

                ThriftyNova motor_follower = new ThriftyNova(followerData[i].id.id());
                ThriftyNova.ThriftyNovaConfig config_follower = new ThriftyNova.ThriftyNovaConfig();

                // Copy config values
                config_follower.motorType = config.motorType;
                config_follower.maxCurrent = config.maxCurrent;
                config_follower.currentType = config.currentType;
                config_follower.voltageCompensation = config.voltageCompensation;
                config_follower.brakeMode = config.brakeMode;

                // Override follower-specific setting
                config_follower.inverted = followerData[i].opposesMain;

                tryUntilOk(
                    motor_follower,
                    5,
                    () -> motor_follower.applyConfig(config_follower)
                );

                tryUntilOk(
                    motor_follower,
                    5,
                    () -> motor_follower.follow(motor.getID())
                );

                followers[i] = motor_follower;
            }

            this.config = config;

            //Defines optional variables for motion profile
            profileState = new TrapezoidProfile.State(motor.getPosition(),0.0);

            double maxVel = maxVelocity
                    .map(v -> v.in(RotationsPerSecond))
                    .orElse(Double.POSITIVE_INFINITY);

            double maxAccel = maxAcceleration
                    .map(a -> a.in(RotationsPerSecondPerSecond))
                    .orElse(Double.POSITIVE_INFINITY);

            kMaxVelocityRps = maxVel;
            lastAccelLimit = maxAccel;
            if (Double.isFinite(maxAccel)) {
                velocityLimiter = new SlewRateLimiter(maxAccel);
            }
            kDefaultConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);

            //Control type idle mode intalization
            idleMode = motor.getBrakeMode() ? ControlType.BRAKE : ControlType.COAST;

    }

    private PIDConfiguration getClosedLoopSlot(PIDSlot slot) {
            switch(slot) {
                case SLOT_0:
                    return config.pid0;

                case SLOT_1:
                    return config.pid1;

                default:
                    throw new IllegalArgumentException("Unsupported PID slot");
            }
    }   
    
    /**
     * Checks if the motor is currently running a position control mode.
     *
     * @return True if the motor is using a position control mode.
     */
    protected boolean isRunningPositionControl() {
        return controlMode == ControlType.POSITION;   
    }

    /**
     * Checks if the motor is currently running a velocity control mode.
     *
     * @return True if the motor is using a velocity control mode.
     */
    protected boolean isRunningVelocityControl() {
        return controlMode == ControlType.VELOCITY;
    }

    /**
     * Checks if the motor is running any Max Motion mode.
     *
     * @return True if the motor is using a Max Motion mode.
     */
    protected boolean isRunningMotionControl() {
        return runningMotionProfile;
    }

    /**
     * Returns the current control type.
     *
     * @return The current control type.
     */
    protected ControlType getCurrentControlType() {
        return  controlMode;
    }


    @Override
    public void updateInputs(MotorInputs inputs) {

        inputs.connected = false;

        ifOk(motor::getPosition, (value) -> {
            inputs.connected = true;
            inputs.position = Rotations.of(value);
        });

        // Velocity
        ifOk(motor::getVelocity,
            (value) -> inputs.velocity = RotationsPerSecond.of(value));

        // Applied voltage
        ifOk(motor::getVoltage,
            (value) -> inputs.appliedVoltage = Volts.of(value));

        // Current
        ifOk(motor::getSupplyCurrent,
            (value) -> inputs.supplyCurrent = Amps.of(value));

        // Temperature
        ifOk(motor::getTemperature,
            (value) -> inputs.temperature = Celsius.of(value));


        // Thrifty Nova does not expose torque current
        inputs.torqueCurrent = Amps.zero();


        // Defaults
        inputs.goalPosition = Rotations.zero();
        inputs.goalVelocity = RotationsPerSecond.zero();
        inputs.positionError = Rotations.zero();
        inputs.velocityError = RotationsPerSecond.zero();
        inputs.activeTrajectoryPosition = Rotations.zero();
        inputs.activeTrajectoryVelocity = RotationsPerSecond.zero();


        // Update control-specific telemetry
        switch (controlMode) {

            case POSITION:
                inputs.goalPosition = goalPosition;

                inputs.positionError =
                        inputs.goalPosition.minus(inputs.position);

                // If using your software trapezoid profile:
                inputs.activeTrajectoryPosition =
                        Rotations.of(profileState.position);

                inputs.activeTrajectoryVelocity =
                        RotationsPerSecond.of(profileState.velocity);

                break;


            case VELOCITY:
                inputs.goalVelocity = goalVelocity;

                inputs.velocityError =
                        inputs.goalVelocity.minus(inputs.velocity);

                break;


            default:
                break;
        }


        if (inputs.connected) {
            inputs.controlType = getCurrentControlType();
        }


        controlMode = idleMode;
    }

    @Override
    public void runCoast() {
        idleMode = ControlType.COAST;
        controlMode = ControlType.COAST;

       if (motor.getBrakeMode()){

        tryUntilOk(
                motor,
                5,
                () -> motor.setBrakeMode(false)
            );
       }
       
       motor.stopMotor();
    }

    @Override
    public void runBrake() {
        idleMode = ControlType.BRAKE;
        controlMode = ControlType.COAST;

        if (!motor.getBrakeMode()){

        tryUntilOk(
                motor,
                5,
                () -> motor.setBrakeMode(true)
            );
       }

        motor.stopMotor();
    }

    @Override
    public void runVoltage(Voltage voltage) {
        controlMode = ControlType.VOLTAGE;
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void runDutyCycle(double dutyCycle, boolean ignoringSoftLimits) {
        controlMode = ControlType.DUTYCYCLE;
        motor.set(dutyCycle);
    }

    @Override
    public void setEncoderPosition(Angle position) {
        motor.setEncoderPosition(position.in(Rotations));
    }

    @Override
    public void runCurrent(Current current) {
        System.out.println("Thrifty Nova does not support current control.");
    }

    @Override
    public void runCurrent(Current current, double dutyCycle) {
        System.out.println("Thrifty Nova does not support current control.");
    }

    @Override
    public void runPosition(Angle position, PIDSlot slot) {
        controlMode = ControlType.POSITION;
        goalPosition = position;

        TrapezoidProfile profile = new TrapezoidProfile(kDefaultConstraints);

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        double goal = position.in(Rotations);
        lastGoal = goal;

        profileState = profile.calculate(
                dt,
                profileState,
                new TrapezoidProfile.State(goal, 0.0));

        motor.setPosition(profileState.position);
        runningMotionProfile = true;

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

        if (goal != lastGoal) {
            lastGoal = goal;
            profileState = new TrapezoidProfile.State(
            motor.getPosition(),
            0.0);
        }

        TrapezoidProfile profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        cruiseVelocity.in(RotationsPerSecond),
                        acceleration.in(RotationsPerSecond.per(Second))));

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        profileState = profile.calculate(
                dt,
                profileState,
                new TrapezoidProfile.State(goal, 0.0));

        motor.setPosition(profileState.position);
        runningMotionProfile = true; 
    }

    @Override
    public void runUnprofiledPosition(Angle position, PIDSlot slot) {
        controlMode = ControlType.POSITION;
        motor.setPosition(position.in(Rotations));
        goalPosition = position;
        runningMotionProfile = false;
    }

    @Override
    public void runVelocity(AngularVelocity velocity, PIDSlot slot) {
        controlMode = ControlType.VELOCITY;
        motor.setVelocity(velocity.in(RotationsPerSecond));
        goalVelocity = velocity;
        runningMotionProfile = false;
    }

    @Override
    public void runVelocity(
            AngularVelocity velocity,
            AngularAcceleration acceleration,
            PIDSlot slot) {

        controlMode = ControlType.VELOCITY;
        goalVelocity = velocity;

        double accelLimit = acceleration.in(RotationsPerSecond.per(Second));
        if (accelLimit != lastAccelLimit) {
            velocityLimiter = new SlewRateLimiter(accelLimit);
            lastAccelLimit = accelLimit;
        }

        double targetVelocity =
                MathUtil.clamp(
                        velocity.in(RotationsPerSecond),
                        -kMaxVelocityRps,
                        kMaxVelocityRps);

        double output = Double.isFinite(lastAccelLimit)
            ? velocityLimiter.calculate(targetVelocity)
            : targetVelocity;

        motor.setVelocity(output);
        runningMotionProfile = true;

    }

    @Override
    public void setPID(PIDSlot slot, PID pid) {

        PIDConfiguration PIDSlot = getClosedLoopSlot(slot);

        PIDSlot.p = pid.P();
        PIDSlot.i = pid.I();
        PIDSlot.d = pid.D();
        PIDSlot.f = pid.V();
    
        tryUntilOk(
            motor,
            5,
            () -> motor.applyConfig(config)
        );

    }

    @Override
    public void setSupplyCurrentLimit(Current currentLimit) {
        motor.setMaxCurrent(CurrentType.SUPPLY, currentLimit.in(Amps));
    }

    @Override
    public int getNumberOfMotors() {
        return (1 + followers.length);
    }

    @Override
    public void close() {

        try {
            motor.close();
                    
            for (ThriftyNova follower : followers) {
                follower.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static boolean thriftyStickyFault = false;

    public static void ifOk(
        DoubleSupplier supplier,
        DoubleConsumer consumer) {

        try {
            consumer.accept(supplier.getAsDouble());
        } catch (Exception e) {
            thriftyStickyFault = true;
        }
    }

    public static void ifOk(
            DoubleSupplier[] suppliers,
            Consumer<double[]> consumer) {

        double[] values = new double[suppliers.length];

        try {
            for (int i = 0; i < suppliers.length; i++) {
                values[i] = suppliers[i].getAsDouble();
            }

            consumer.accept(values);

        } catch (Exception e) {
            thriftyStickyFault = true;
        }
    }

    /** Attempts to run the command until the Nova reports no errors. */
    public static void tryUntilOk(
            ThriftyNova motor,
            int maxAttempts,
            Runnable command) {

        for (int i = 0; i < maxAttempts; i++) {

            motor.clearErrors();

            command.run();

            if (motor.getErrors().isEmpty()) {
                return;
            }
        }

        thriftyStickyFault = true;
    }



}