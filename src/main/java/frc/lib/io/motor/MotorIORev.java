package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import java.util.EnumMap;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.Device;
import frc.lib.util.PID;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Abstraction for a REV motor implementing the {@link MotorIO} interface. Wraps motor
 * setup, control modes.
 */
public class MotorIORev implements MotorIO {

    /**
     * Configuration data for a REV motor that follows another REV motor.
     * 
     * <p>
     * Follower motors mirror the output of a main motor, useful for mechanisms that require
     * multiple motors working together (like a dual-motor elevator).
     * 
     * @param id The CAN device ID of the follower motor
     * @param opposesMain Whether this follower should spin opposite to the main motor
     */
    public record RevFollower(Device.CAN id, boolean opposesMain) {}

    protected final SparkBase motor;
    protected final SparkBase[] followers;

    protected final SparkClosedLoopController controller;
    protected final RelativeEncoder encoder;
    protected final boolean isFlex;

    // Define REV control types
    protected IdleMode currentIdleMode;
    protected final com.revrobotics.spark.SparkBase.ControlType voltageControl = com.revrobotics.spark.SparkBase.ControlType.kVoltage;
    protected final com.revrobotics.spark.SparkBase.ControlType currentControl =  com.revrobotics.spark.SparkBase.ControlType.kCurrent;
    protected final com.revrobotics.spark.SparkBase.ControlType dutyCycleControl = com.revrobotics.spark.SparkBase.ControlType.kDutyCycle;
    protected final com.revrobotics.spark.SparkBase.ControlType positionControl = com.revrobotics.spark.SparkBase.ControlType.kMAXMotionPositionControl;
    protected final com.revrobotics.spark.SparkBase.ControlType unprofiledPositionControl = com.revrobotics.spark.SparkBase.ControlType.kPosition;
    protected final com.revrobotics.spark.SparkBase.ControlType velocityControl = com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl;
    protected final com.revrobotics.spark.SparkBase.ControlType unprofiledVelocityControl = com.revrobotics.spark.SparkBase.ControlType.kVelocity;

    // Caches for last-applied MAX Motion parameters (NaN = never applied).
    // Compare these with Double.compare, never ==, so that the "never applied" NaN state does not
    // silently defeat the cache: NaN == NaN is false in Java, which would make every call
    // reconfigure the motor over CAN.
    private double lastRequestedMaxMotionVelocity = Double.NaN;
    private double lastRequestedMaxMotionAcceleration = Double.NaN;

    /** Warns if code asks for a duty cycle cap on current control, which REV cannot do. */
    private final Alert currentDutyCycleUnsupportedAlert;

    /** Warns if code asks to bypass soft limits, which REV cannot do per-request. */
    private final Alert softLimitOverrideUnsupportedAlert;

    /**
     * Constructs and initializes a REV motor.
     * 
     * <p>
     * This constructor applies the provided configuration to the main motor and all followers. It
     * sets up the follower relationship, initializes telemetry, and configures
     * encoder. All followers must be on the same CAN bus as the main motor.
     * 
     * @param name The name of the motor(s) for logging and identification
     * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios. 
     * <b> Note: pass though a SparkFlex or SparkMax config or else max motion will not behave as expected</b>
     * @param CAN the CAN device reference containing the motor's CAN ID
     * @param followerData Configuration data for the follower motor(s), can be empty if no
     *        followers {@link RevFollower}
     */
    public MotorIORev(
        String name,
        SparkBaseConfig config,
        Device.CAN CAN,
        RevFollower... followerData)
    {

        currentDutyCycleUnsupportedAlert =
                new Alert(
                        name
                                + ": runCurrent(current, dutyCycle) ignores the duty cycle limit on"
                                + " REV motors. The current was applied, but top speed is not"
                                + " capped.",
                        AlertType.kWarning);

        softLimitOverrideUnsupportedAlert =
                new Alert(
                        name
                                + ": runDutyCycle(dutyCycle, ignoringSoftLimits) cannot bypass soft"
                                + " limits on REV motors. The configured soft limits are still"
                                + " being enforced.",
                        AlertType.kWarning);

        // Initialize motor based on whether it's flex or not
        if (config instanceof SparkFlexConfig) {
            isFlex = true;
            motor = new SparkFlex(CAN.id(), MotorType.kBrushless);
        } else {
            isFlex = false;
            motor = new SparkMax(CAN.id(), MotorType.kBrushless);
        }

        //Applies config to the motor
        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );

        //Define controller and encoder
        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        followers = new SparkBase[followerData.length];

        //For loop that iterates for each follower mototr
         for (int i = 0; i < followerData.length; i++) {
            SparkBase motor_follower;
            SparkBaseConfig config_follower;
    
            // Initialize motor and new config based on whether it's flex or not
            if (isFlex) {
                motor_follower = new SparkFlex(followerData[i].id.id(), MotorType.kBrushless);
                config_follower = new SparkFlexConfig();
            } else {
                motor_follower = new SparkMax(followerData[i].id.id(), MotorType.kBrushless);
                config_follower = new SparkMaxConfig();
            }   
    
            //Defines the config to follow main motor
            config_follower
            .apply(config)
            .follow(CAN.id(), followerData[i].opposesMain);
    
            //Applies config to follower motor
            tryUntilOk(
                motor_follower,
                5,
                () -> motor_follower.configure(
                    config_follower,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                )
            );

            followers[i] = motor_follower;
        }
    }


    /**
     * Maps {@link MotorIO.PIDSlot} to REV {@link ClosedLoopSlot}.
     */
    private static final EnumMap<MotorIO.PIDSlot, ClosedLoopSlot> SLOT_MAP =
        new EnumMap<>(MotorIO.PIDSlot.class);
    static {
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_0, ClosedLoopSlot.kSlot0);
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_1, ClosedLoopSlot.kSlot1);
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_2, ClosedLoopSlot.kSlot2);
        SLOT_MAP.put(MotorIO.PIDSlot.SLOT_3, ClosedLoopSlot.kSlot3);
    }

    /**
     * Gets the corresponding REV {@link ClosedLoopSlot} for a given {@link MotorIO.PIDSlot}.
     * 
     * @param slot The MotorIO.PIDSlot to convert.
     */
    protected ClosedLoopSlot getClosedLoopSlot(MotorIO.PIDSlot slot)
    {
        return SLOT_MAP.getOrDefault(slot, ClosedLoopSlot.kSlot0);
    }

    /**
     * Checks if the motor is currently running a position control mode.
     *
     * @return True if the motor is using a position control mode.
     */
    protected boolean isRunningPositionControl() {
        var control =  motor.getClosedLoopController().getControlType();
        return (control == com.revrobotics.spark.SparkBase.ControlType.kPosition)
                || (control == com.revrobotics.spark.SparkBase.ControlType.kMAXMotionPositionControl);
    }

    /**
     * Checks if the motor is currently running a velocity control mode.
     *
     * @return True if the motor is using a velocity control mode.
     */
    protected boolean isRunningVelocityControl() {
        var control = motor.getClosedLoopController().getControlType();
        return (control == com.revrobotics.spark.SparkBase.ControlType.kVelocity)
                || (control == com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Checks if the motor is running any Max Motion mode.
     *
     * @return True if the motor is using a Max Motion mode.
     */
    protected boolean isRunningMaxMotionControl() {
        var control = motor.getClosedLoopController().getControlType();
        return (control == com.revrobotics.spark.SparkBase.ControlType.kMAXMotionPositionControl)
                || (control == com.revrobotics.spark.SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Returns the current control type.
     *
     * @return The current control type.
     */
    protected ControlType getCurrentControlType() {
        var control = motor.getClosedLoopController().getControlType();

        if (control == currentControl) {
            return ControlType.CURRENT;
        } else if (control == voltageControl) {
            return ControlType.VOLTAGE;
        } else if (control == dutyCycleControl) {
            return ControlType.DUTYCYCLE;
        } else if (isRunningPositionControl()) {
            return ControlType.POSITION;
        } else if (isRunningVelocityControl()) {
            return ControlType.VELOCITY;
        } else if (currentIdleMode == IdleMode.kCoast){
        return ControlType.COAST;
        }else if (currentIdleMode == IdleMode.kBrake){
        return ControlType.BRAKE;
        }

        return ControlType.COAST;
    }

    /**
     * Updates the passed-in MotorInputs structure with the latest sensor readings.
     *
     * @param inputs Motor input structure to populate.
     */
    @Override
    public void updateInputs(MotorInputs inputs)
    {
      
        inputs.connected = false; // default each loop

        //Updated connection to true if connected
        ifOk(motor, motor::getBusVoltage, (voltage) -> {
            inputs.connected =
                motor.getLastError() == com.revrobotics.REVLibError.kOk
                && voltage > 0.0;
        });

        ifOk(motor, encoder::getPosition,
            (value) -> inputs.position = Rotation.of(value));

        ifOk(motor, encoder::getVelocity,
            (value) -> inputs.velocity = RPM.of(value));

        ifOk(motor,
            new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
            (values) -> inputs.appliedVoltage =
                Volts.of(values[0] * values[1]));

        ifOk(motor, motor::getOutputCurrent,
            (value) -> inputs.supplyCurrent = Amps.of(value));

        // REVLib exposes no torque-producing current signal. Log NaN rather than null: null is
        // silently dropped when writing logs and throws when reading them back during replay.
        inputs.torqueCurrent = Amps.of(UNSUPPORTED_SIGNAL);

        ifOk(motor, motor::getMotorTemperature,
            (value) -> inputs.temperature = Celsius.of(value));


        boolean isRunningPositionControl = isRunningPositionControl();
        boolean isRunningMaxMotion = isRunningMaxMotionControl();
        boolean isRunningVelocityControl = isRunningVelocityControl();

        // Default everything to zero
        inputs.goalPosition = Rotations.zero();
        inputs.goalVelocity = RPM.zero();
        inputs.positionError = Rotations.zero();
        inputs.velocityError = RPM.zero();
        inputs.activeTrajectoryPosition = Rotations.zero();
        inputs.activeTrajectoryVelocity = RPM.zero();

        if (isRunningPositionControl) {
        ifOk(
            motor,
            () -> motor.getClosedLoopController().getSetpoint(),
            value -> inputs.goalPosition = Rotations.of(value));

        if (isRunningMaxMotion) {
            ifOk(
                motor,
                () -> motor.getClosedLoopController().getMAXMotionSetpointPosition(),
                value -> inputs.activeTrajectoryPosition = Rotations.of(value));

            ifOk(
                motor,
                () -> motor.getClosedLoopController().getMAXMotionSetpointVelocity(),
                value -> inputs.activeTrajectoryVelocity =
                    RPM.of(value));
        }

        inputs.positionError =
            inputs.goalPosition.minus(inputs.position);
        }
        else if (isRunningVelocityControl) {
            ifOk(
                motor,
                () -> motor.getClosedLoopController().getSetpoint(),
                value -> inputs.goalVelocity =
                    RPM.of(value));

            if (isRunningMaxMotion) {
                ifOk(
                    motor,
                    () -> motor.getClosedLoopController().getMAXMotionSetpointVelocity(),
                    value -> inputs.activeTrajectoryVelocity =
                        RPM.of(value));
            }

            inputs.velocityError =
                inputs.goalVelocity.minus(inputs.velocity);
        }

        if (inputs.connected) {
            inputs.controlType = getCurrentControlType();
        }
        
    }

    @Override
    public void runCoast()
    {
    if (currentIdleMode != IdleMode.kCoast) {
        SparkBaseConfig config = newEmptyConfig();
        config.idleMode(IdleMode.kCoast);

            tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                    config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters
                )
            );

        currentIdleMode = IdleMode.kCoast;
    }

        motor.stopMotor();
    }
 
    @Override
    public void runBrake()
    {
    if (currentIdleMode != IdleMode.kBrake) {
        SparkBaseConfig config = newEmptyConfig();
        config.idleMode(IdleMode.kBrake);

        motor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        currentIdleMode = IdleMode.kBrake;
    }

        motor.stopMotor();
    }

    @Override
    public void runVoltage(Voltage volts)
    {
        controller.setSetpoint(volts.in(Volts), voltageControl);
    }

    @Override
    public void runCurrent(Current current)
    {
        controller.setSetpoint(current.in(Amps), currentControl);
    }

    /**
     * Runs the motor at a target current.
     *
     * <p><b>REV limitation:</b> a SPARK cannot cap the duty cycle of a current request the way a
     * TalonFX can, so {@code dutyCycle} is not enforced. The current is still applied, and a
     * dashboard warning is raised so this does not fail silently.
     *
     * @param current Desired torque-producing current
     * @param dutyCycle Ignored on REV motors
     */
    @Override
    public void runCurrent(Current current, double dutyCycle)
    {
        currentDutyCycleUnsupportedAlert.set(dutyCycle < 1.0);

        runCurrent(current);
    }

    /**
     * Runs the motor using duty cycle (percentage of available voltage).
     *
     * <p><b>REV limitation:</b> a SPARK has no per-request soft limit override, so
     * {@code ignoringSoftLimits} is not honored. Whatever the config set up stays in effect.
     * Supporting it would mean writing a new config over CAN and restoring the configured state
     * afterwards; until that exists, a dashboard warning is raised so this does not fail silently.
     *
     * @param dutyCycle Fractional output between -1 and 1
     * @param ignoringSoftLimits Ignored on REV motors
     */
    @Override
    public void runDutyCycle(double dutyCycle, boolean ignoringSoftLimits)
    {
        softLimitOverrideUnsupportedAlert.set(ignoringSoftLimits);

        motor.set(MathUtil.clamp(dutyCycle, -1.0, 1.0));
    }

    @Override
    public void runPosition(Angle position, PIDSlot slot){
        controller.setSetpoint(position.in(Rotation), positionControl,
            getClosedLoopSlot(slot));
    }

    @Override
    public  void runPosition( Angle position, PIDSlot slot, AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {

        double newCruise = cruiseVelocity.in(RPM);
        double newAccel = acceleration.in(RotationsPerSecondPerSecond)*60; //Convert RPS per second to RPM per second

        queueMaxMotionConfigUpdate(newCruise, newAccel);

        controller.setSetpoint(position.in(Rotation), positionControl,
            getClosedLoopSlot(slot));
    }

    @Override
    public void runUnprofiledPosition(Angle position, PIDSlot slot) {
        controller.setSetpoint(position.in(Rotation), unprofiledPositionControl,
            getClosedLoopSlot(slot));
    }

    @Override
    public void runVelocity(AngularVelocity velocity, PIDSlot slot){
        controller.setSetpoint(velocity.in(RPM),
            unprofiledVelocityControl,
            getClosedLoopSlot(slot));
    }


    @Override
    public void runVelocity(AngularVelocity velocity, AngularAcceleration acceleration,
        PIDSlot slot){
        double newAccel = acceleration.in(RotationsPerSecondPerSecond)*60; //Convert RPS per second to RPM per second

        // Only the acceleration is being changed here. Passing the cached cruise velocity back in
        // would write NaN to the motor before any position request has ever set one.
        queueMaxMotionAccelerationUpdate(newAccel);

        controller.setSetpoint(velocity.in(RPM),
            velocityControl,
            getClosedLoopSlot(slot));
    }

    @Override
    public void setEncoderPosition(Angle position)
    {
        encoder.setPosition(position.in(Rotation));

    }

    /**
     * Applies new MAX Motion cruise velocity and acceleration limits, skipping the write if the
     * motor already has these values.
     *
     * @param maxVelocity Cruise velocity, in the RPM
     * @param maxAcceleration Acceleration, in the units RPM per second
     */
    private void queueMaxMotionConfigUpdate(double maxVelocity, double maxAcceleration) {
        if (Double.compare(maxVelocity, lastRequestedMaxMotionVelocity) == 0
                && Double.compare(maxAcceleration, lastRequestedMaxMotionAcceleration) == 0) {
            return;
        }

        lastRequestedMaxMotionVelocity = maxVelocity;
        lastRequestedMaxMotionAcceleration = maxAcceleration;

        SparkBaseConfig config = newEmptyConfig();

        config.closedLoop.maxMotion
            
            .cruiseVelocity(maxVelocity)
            .maxAcceleration(maxAcceleration);

        applyMaxMotionConfig(config);
    }

    /**
     * Applies a new MAX Motion acceleration limit, leaving the cruise velocity untouched.
     *
     * @param maxAcceleration Acceleration, in RPM per second
     */
    private void queueMaxMotionAccelerationUpdate(double maxAcceleration) {
        if (Double.compare(maxAcceleration, lastRequestedMaxMotionAcceleration) == 0) {
            return;
        }

        lastRequestedMaxMotionAcceleration = maxAcceleration;

        SparkBaseConfig config = newEmptyConfig();
        config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);

        applyMaxMotionConfig(config);
    }

    private void applyMaxMotionConfig(SparkBaseConfig config) {
            tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                    config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters
                )
            );
    }


    @Override
    public void setPID(PIDSlot slot, PID pid) {

        SparkBaseConfig config = newEmptyConfig();

        ClosedLoopSlot revSlot = SLOT_MAP.get(slot);

        config.closedLoop
            .p(pid.P(), revSlot)
            .i(pid.I(), revSlot)
            .d(pid.D(), revSlot)
            .feedForward
            .kA(pid.A(), revSlot)
            .kV(pid.V(), revSlot)
            .kG(pid.G(), revSlot)
            .kS(pid.S(), revSlot);

        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            )
        );
    }

    @Override
    public void setSupplyCurrentLimit(Current currentLimit) {

        SparkBaseConfig config = newEmptyConfig();

        config.smartCurrentLimit((int) currentLimit.in(Amps));

        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            )
        );
    }

    @Override
    public  int getNumberOfMotors() {
        return followers.length + 1;
    }

    @Override
    public void close()
    {
        motor.close();

        for (SparkBase follower : followers) {
            follower.close();
        }
    }

    /**
     * @return SparkMax or SparkFlex config based on the original config
     */
    private SparkBaseConfig newEmptyConfig()
    {
        return (motor instanceof SparkFlex) ? new SparkFlexConfig() : new SparkMaxConfig();
    }

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

     /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

}
