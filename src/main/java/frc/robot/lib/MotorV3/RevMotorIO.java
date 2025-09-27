package frc.robot.lib.MotorV3;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.lib.Motor.MotionProfileHelpers;
import frc.robot.lib.Motor.MotorTypes;
import frc.robot.lib.Motor.MotionProfileHelpers.REV_MotionProfile;
import frc.robot.lib.Motor.MotorTypes.Motor_Type;

public class RevMotorIO implements MotorIO {

    private SparkBase motor;
    private SparkClosedLoopController controller;
    private SparkBaseConfig config;
    private RelativeEncoder encoder;
    private boolean isFlex;

    private final int id;
    private final double gearRatio;
    private final double wheelRadius;

    /**
     * Constructor for a standalone leader motor. This motor is configured as the primary motor 
     * in a system and can be manually controlled by the user. It also configures various settings 
     * for motion profiling and closed-loop control.
     *
     * @param id The unique identifier for the motor (CAN ID).
     * @param isFlex Boolean indicating whether the motor is a SparkFlex (true) or a SparkMax (false).
     * @param gearRatio The gear ratio applied to the motor's output.
     * @param wheelRadius The radius of the wheel attached to the motor, used for motion calculations.
     * @param isBrake Boolean indicating whether the motor should use brake mode (true) or coast mode (false).
     * @param inverted Boolean indicating whether the motor should be inverted relative to the control signal.
     * @param motorType The type of motor (e.g., brushless, brushed) to configure.
     * @param motionProfile The motion profile settings for the motor, including PID and velocity parameters.
     *
     * @see MotorIO
     */
    public RevMotorIO(
        int id,
        boolean isFlex,
        double gearRatio,
        double wheelRadius,
        boolean isBrake,
        boolean inverted,
        Motor_Type motorType,
        REV_MotionProfile motionProfile
    ) {
        this.id = id;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.isFlex=isFlex;
        motorType = (motorType != null) ? motorType : new MotorTypes.REV_Defualt();
        motionProfile = (motionProfile != null) ? motionProfile : MotionProfileHelpers.getDisabledREVProfile();

        if (isFlex) {
            motor = new SparkFlex(id, MotorType.kBrushless);
            config = new SparkFlexConfig();
        } else {
            motor = new SparkMax(id, MotorType.kBrushless);
            config = new SparkMaxConfig();
        }

        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        config.smartCurrentLimit(
            motorType.getDefaultAmpLimits()[0],
            motorType.getDefaultAmpLimits()[1],
            motorType.getDefaultAmpLimits()[2]
        )
        .inverted(inverted)
        .idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast)
        .signals.primaryEncoderPositionPeriodMs(20);
        
        config.closedLoop
            // Position slot 0
            .p(motionProfile.kP_Pos, ClosedLoopSlot.kSlot0)
            .i(motionProfile.kI_Pos, ClosedLoopSlot.kSlot0)
            .d(motionProfile.kD_Pos, ClosedLoopSlot.kSlot0)
            .iZone(motionProfile.iZone_Pos, ClosedLoopSlot.kSlot0)
            .iMaxAccum(motionProfile.iMaxAccum_Pos, ClosedLoopSlot.kSlot0)
            .outputRange(motionProfile.outMin_Pos, motionProfile.outMax_Pos, ClosedLoopSlot.kSlot0)

            // Velocity slot 1
            .p(motionProfile.kP_Vel, ClosedLoopSlot.kSlot1)
            .i(motionProfile.kI_Vel, ClosedLoopSlot.kSlot1)
            .d(motionProfile.kD_Vel, ClosedLoopSlot.kSlot1)
            .velocityFF(motionProfile.kV_Vel, ClosedLoopSlot.kSlot1)
            .iZone(motionProfile.iZone_Vel, ClosedLoopSlot.kSlot1)
            .iMaxAccum(motionProfile.iMaxAccum_Vel, ClosedLoopSlot.kSlot1)
            .outputRange(motionProfile.outMin_Vel, motionProfile.outMax_Vel, ClosedLoopSlot.kSlot1)

            // Alternate slot 2
            .pidf(motionProfile.kP_Alt, motionProfile.kI_Alt, motionProfile.kD_Alt, motionProfile.kV_Alt, ClosedLoopSlot.kSlot2)
            .iZone(motionProfile.iZone_Alt, ClosedLoopSlot.kSlot2)
            .iMaxAccum(motionProfile.iMaxAccum_Alt, ClosedLoopSlot.kSlot2)
            .outputRange(motionProfile.outMin_Alt, motionProfile.outMax_Alt, ClosedLoopSlot.kSlot2);

        config.closedLoop.maxMotion
            .maxVelocity(motionProfile.cruiseRPM, ClosedLoopSlot.kSlot0)
            .maxAcceleration(motionProfile.accelRPMPerSec, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(motionProfile.allowedErrorRotations, ClosedLoopSlot.kSlot0)
            .maxVelocity(motionProfile.cruiseRPM, ClosedLoopSlot.kSlot1)
            .maxAcceleration(motionProfile.accelRPMPerSec, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(motionProfile.allowedErrorRotations, ClosedLoopSlot.kSlot1);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Constructor for a motor follower. This motor is configured to follow the leader motor, 
     * and should not be manually controlled by the user. The follower motor adopts the leader's 
     * configuration settings, including current limits, gear ratio, and motion profiling.
     *
     * @param id The unique identifier for the motor (CAN ID).
     * @param isFlex Boolean indicating whether the motor is a SparkFlex (true) or a SparkMax (false).
     * @param isBrake Boolean indicating whether the motor should use brake mode (true) or coast mode (false).
     * @param inverted_from_leader Boolean indicating whether the motor should be inverted relative to the leader motor.
     *
     */
    @Override
    public void set_Follower(
        int id,
        boolean isBrake,
        boolean inverted_from_leader
    ) {

        SparkBase motor_follower;
        SparkBaseConfig config_follower;

        // Initialize motor and config based on whether it's flex or not
        if (isFlex) {
            motor_follower = new SparkFlex(id, MotorType.kBrushless);
            config_follower = new SparkFlexConfig();
        } else {
            motor_follower = new SparkMax(id, MotorType.kBrushless);
            config_follower = new SparkMaxConfig();
        }

        config_follower
        .apply(config)
        .idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast)
        .follow(this.id, inverted_from_leader);
        controller = motor_follower.getClosedLoopController();

        motor_follower.configure(config_follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.motorPositionRot = encoder.getPosition();
        inputs.motorVelocityRPM = encoder.getVelocity();
        inputs.mechanismPositionRot = inputs.motorPositionRot / gearRatio;
        inputs.mechanismVelocityRPM = inputs.motorVelocityRPM / gearRatio;
        inputs.tipSpeedMps = 2 * Math.PI * wheelRadius * (inputs.mechanismVelocityRPM / 60.0);
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.amps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        controller.setReference(volts, ControlType.kVoltage, ClosedLoopSlot.kSlot2);
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setTargetMotorPosition(double motorRotations) {
        controller.setReference(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setCurrentMotorPosition(double motorRotations) {
        encoder.setPosition(motorRotations);
    }

    @Override
    public void setTargetMechanismPosition(double mechanismRotations) {
        setTargetMotorPosition(mechanismRotations * gearRatio);
    }

    @Override
    public void setCurrentMechanismPosition(double mechanismRotations) {
        setCurrentMotorPosition(mechanismRotations * gearRatio);
    }

    @Override
    public void setTargetMotorVelocity(double motorRPM) {
        controller.setReference(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }


    @Override
    public void setTargetMechanismVelocity(double mechanismRPM) {
        setTargetMotorVelocity(mechanismRPM * gearRatio);
    }

    @Override
    public void setTargetTipSpeed(double mps) {
        double mechRPM = (mps * 60) / (2 * Math.PI * wheelRadius);
        setTargetMechanismVelocity(mechRPM);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        SparkBaseConfig newConfig = newEmptyConfig();
        newConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTunerConstants(double p, double i, double d, double ff,
                                  double iZone, double iMaxAccum,
                                  double minOutput, double maxOutput, int slot) {
        SparkBaseConfig newConfig = newEmptyConfig();
        ClosedLoopSlot clSlot = getSlot(slot);

        newConfig.closedLoop
            .p(p, clSlot)
            .i(i, clSlot)
            .d(d, clSlot)
            .velocityFF(ff, clSlot)
            .iZone(iZone, clSlot)
            .iMaxAccum(iMaxAccum, clSlot)
            .outputRange(minOutput, maxOutput, clSlot);

        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setMotionConstraints(double maxVel, double maxAccel,
                                     double allowedError, int slot) {
        SparkBaseConfig newConfig = newEmptyConfig();
        ClosedLoopSlot clSlot = getSlot(slot);

        newConfig.closedLoop.maxMotion
            .maxVelocity(maxVel, clSlot)
            .maxAcceleration(maxAccel, clSlot)
            .allowedClosedLoopError(allowedError, clSlot);

        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
        SparkBaseConfig newConfig = newEmptyConfig();
        newConfig.smartCurrentLimit(stallLimit, freeLimit, limitRPM);
        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setMotionLimit(Double max, Double min) {
        SparkBaseConfig newConfig = newEmptyConfig();
    
        if (max != null) {
            newConfig.softLimit
                     .forwardSoftLimit(max)              
                     .forwardSoftLimitEnabled(true);
        }
    
        if (min != null) {
            newConfig.softLimit
                     .reverseSoftLimit(min)
                     .reverseSoftLimitEnabled(true);
        }

        motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private SparkBaseConfig newEmptyConfig() {
        return (motor instanceof SparkFlex) ? new SparkFlexConfig() : new SparkMaxConfig();
    }

    private ClosedLoopSlot getSlot(int slotNumber) {
        return switch (slotNumber) {
            case 0 -> ClosedLoopSlot.kSlot0;
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> {
                System.err.println("Invalid PID slot: " + slotNumber);
                yield ClosedLoopSlot.kSlot0;
            }
        };
    }
}
