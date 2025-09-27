package frc.robot.lib.MotorV3;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.Motor.MotionProfileHelpers.CTRE_MotionProfile;
import frc.robot.lib.Motor.MotorTypes.Motor_Type;

public class TalonMotorIO implements MotorIO {

    private final TalonFX motor;
    private final TalonFXConfiguration config;
    private final double gearRatio;
    private final double wheelRadius;
    private final String canBus;

    /**
     * Initializes a Talon motor for standalone use, applying various configurations for motor behavior, including
     * current limits, gear ratio, motion profile, and neutral mode. This constructor is used for motors that 
     * are not following another motor.
     * 
     * @param id The unique motor ID for this Talon motor.
     * @param canBus The CAN bus name to which this motor is connected.
     * @param gearRatio The gear ratio for the motor's mechanism, adjusting the sensor feedback to real-world measurements.
     * @param wheelRadius The radius of the wheel that is being driven by the motor.
     * @param isBrake Specifies whether the motor should engage brake mode (true) or coast mode (false) when not moving.
     * @param inverted Specifies whether the motor's rotation is inverted relative to the normal rotation.
     * @param motorType The type of motor being used, which can define specific amp limits for current control. This should
     *                  not be null if custom current limits are required.
     * @param motionProfile The motion profile to be used by the motor, defining factors such as velocity and acceleration.
     *                      This can be null if motion control is not needed.
     *
     * @see MotorIO
     */
    public TalonMotorIO(
        int id,
        String canBus,
        double gearRatio,
        double wheelRadius,
        boolean isBrake,
        boolean inverted,
        Motor_Type motorType,
        CTRE_MotionProfile motionProfile
    ) {
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.canBus = canBus;
        NeutralModeValue neutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        motor = new TalonFX(id, canBus);
        config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = inverted ?
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        motor.setNeutralMode(neutralMode);

        if (motorType != null) {
            int[] amps = motorType.getDefaultAmpLimits();
            config.CurrentLimits.SupplyCurrentLimit = amps[0];
            config.CurrentLimits.SupplyCurrentLowerTime = amps[1];
            config.CurrentLimits.SupplyCurrentLowerLimit = amps[2];
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = amps[3];
            config.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        config.Feedback.SensorToMechanismRatio = gearRatio;

        if (motionProfile != null) {
            var slot0 = config.Slot0;
            slot0.kS = motionProfile.ff_kS;
            slot0.kV = motionProfile.ff_kV;
            slot0.kA = motionProfile.ff_kA;
            slot0.kP = motionProfile.kP_Pos;
            slot0.kI = motionProfile.kI_Pos;
            slot0.kD = motionProfile.kD_Pos;
            if (motionProfile.gravityType != null) {
                slot0.GravityType = motionProfile.gravityType;
            }

            var slot1 = config.Slot1;
            slot1.kS = motionProfile.velFF_kS;
            slot1.kV = motionProfile.velFF_kV;
            slot1.kA = motionProfile.velFF_kA;
            slot1.kP = motionProfile.kP_Vel;
            slot1.kI = motionProfile.kI_Vel;
            slot1.kD = motionProfile.kD_Vel;

            var motionMagic = config.MotionMagic;
            motionMagic.MotionMagicCruiseVelocity = motionProfile.cruiseVelocity;
            motionMagic.MotionMagicAcceleration = motionProfile.acceleration;
            motionMagic.MotionMagicJerk = motionProfile.jerk;
            motionMagic.MotionMagicExpo_kV = motionProfile.motionKV;
            motionMagic.MotionMagicExpo_kA = motionProfile.motionKA;
        }

        motor.getConfigurator().apply(config, 0.010);
    }

    /**
     * Initializes a Talon motor to follow another leader motor, applying settings like current limits and gear ratio 
     * from the leader. This motor will mirror the leader motor's behavior with an option for inversion.
     * 
     * @param id The unique motor ID for this follower Talon motor.
     * @param canBus The CAN bus name to which this motor is connected.
     * @param isBrake Specifies whether the motor should engage brake mode (true) or coast mode (false) when not moving.
     * @param invertedFromLeader Specifies whether this follower motor should be inverted relative to the leader motor's rotation.
     *
     * @see MotorIO
     */
    @Override
    public void set_Follower(
        int id,
        boolean isBrake,
        boolean invertedFromLeader
    ) {

        //Defines follower motor
        TalonFX motor_follower = new TalonFX(id, canBus);

        //Converts from boolean to Talon neutral mode
        NeutralModeValue neutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //Sets neutral mode
        motor_follower.setNeutralMode(neutralMode);
        
        // Set the motor to follow the leader with inversion
        motor_follower.setControl(new Follower(motor.getDeviceID(), invertedFromLeader));

        // Apply configuration
        motor_follower.getConfigurator().apply(config, 0.010);
    }


    @Override
    public void updateInputs(MotorIOInputs inputs) {
        double mechPos = motor.getPosition().getValueAsDouble();
        double mechVel = motor.getVelocity().getValueAsDouble();

        inputs.motorPositionRot = mechPos * gearRatio;
        inputs.motorVelocityRPM = mechVel * gearRatio * 60.0;
        inputs.mechanismPositionRot = mechPos;
        inputs.mechanismVelocityRPM = mechVel * 60.0;
        inputs.tipSpeedMps = inputs.mechanismVelocityRPM * 2 * Math.PI * wheelRadius / 60.0;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.amps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setTargetMotorPosition(double motorRotations) {
        var control = new MotionMagicVoltage(0).withPosition(motorRotations / gearRatio);
        motor.setControl(control);
    }

    @Override
    public void setCurrentMotorPosition(double motorRotations) {
        motor.setPosition(motorRotations * gearRatio);
    }

    @Override
    public void setTargetMechanismPosition(double mechanismRotations) {
        var control = new MotionMagicVoltage(0).withPosition(mechanismRotations);
        motor.setControl(control);
    }

    @Override
    public void setCurrentMechanismPosition(double mechanismRotations) {
        motor.setPosition(mechanismRotations);
    }

    @Override
    public void setTargetMotorVelocity(double motorRPM) {
        var control = new VelocityVoltage(0).withVelocity(motorRPM / gearRatio / 60.0);
        motor.setControl(control);
    }

    @Override
    public void setTargetMechanismVelocity(double mechanismRPM) {
        var control = new VelocityVoltage(0).withVelocity(mechanismRPM / 60.0);
        motor.setControl(control);
    }

    @Override
    public void setTargetTipSpeed(double mps) {
        setTargetMechanismVelocity(mps / (2 * Math.PI * wheelRadius) * 60.0);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        NeutralModeValue mode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(mode);
    }

    @Override
    public void setTunerConstants(double p, double i, double d, double ff,
                                  double iZone, double iMaxAccum,
                                  double minOutput, double maxOutput, int slot) {
        if (slot == 0) {
            var slot0 = config.Slot0;
            slot0.kP = p;
            slot0.kI = i;
            slot0.kD = d;
            slot0.kS = ff;
        } else if (slot == 1) {
            var slot1 = config.Slot1;
            slot1.kP = p;
            slot1.kI = i;
            slot1.kD = d;
            slot1.kS = ff;
        }
        motor.getConfigurator().apply(config, 0.010);
    }

    @Override
    public void setMotionConstraints(double maxVel, double maxAccel,
                                     double allowedError, int slot) {
        var motionMagic = config.MotionMagic;
        motionMagic.MotionMagicCruiseVelocity = maxVel;
        motionMagic.MotionMagicAcceleration = maxAccel;
        motionMagic.MotionMagicJerk = allowedError;
        motor.getConfigurator().apply(config, 0.010);
    }

    @Override
    public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
        var currLimits = config.CurrentLimits;
        currLimits.SupplyCurrentLimit = stallLimit;
        currLimits.SupplyCurrentLowerLimit = freeLimit;
        currLimits.StatorCurrentLimit = limitRPM;
        currLimits.SupplyCurrentLimitEnable = true;
        currLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(config, 0.010);
    }

    @Override
    public void setMotionLimit(Double max, Double min) {
        TalonFXConfiguration config = new TalonFXConfiguration();
    
        if (max != null) {
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        }
    
        if (min != null) {
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
    
        // Apply with a 10ms timeout. Implementing class must expose 'motor'.
        // If motor reference is not available here, override in the implementing class.
        motor.getConfigurator().apply(config, 0.010);
    }

    /**
     * This method is used to assits the follower motor in copying the leader config
     * @return the config of the motor
     */
    public TalonFXConfiguration getConfig(){
        return config;
    }

}
