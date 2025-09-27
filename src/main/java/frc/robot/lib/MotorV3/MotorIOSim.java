// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.MotorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.lib.Motor.MotionProfileHelpers.SIM_MotionProfile;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * Motor simulation for DC motors, supporting position and velocity control modes.
 * This class uses a PID controller, feedforward, and trapezoidal motion profiles to simulate a DC motor's behavior.
 */
public class MotorIOSim implements MotorIO {

    private static final double TWO_PI = 2.0 * Math.PI; // Constant for 2π, used for radian conversions
    private static final double DT = 0.02; // 20ms update period, simulating the time step for updates
    private static final double VELOCITY_THRESHOLD = 1e-6; // Small velocity threshold for zero-crossing (RPM)

    // Motor simulation components
    private final DCMotorSim motorSim; // Simulation model for the DC motor
    private final double gearRatio; // Gear ratio for the motor system
    private final double wheelRadiusM; // Radius of the wheel attached to the motor (in meters)
    private double appliedVolts = 0.0; // Voltage applied to the motor
    private boolean isClosedLoop = false; // Flag to enable/disable closed-loop control
    private boolean isVelocityControl = false; // Control mode flag (velocity vs position)

    // Motion control components
    private PIDController pidControllerPos; // PID controller for position control (rotations)
    private PIDController pidControllerVel; // PID controller for velocity control (RPM)
    private SimpleMotorFeedforward feedforwardPos; // Feedforward controller for position control (volts per RPM)
    private SimpleMotorFeedforward feedforwardVel; // Feedforward controller for velocity control (volts per RPM)
    private TrapezoidProfile profile; // Trapezoidal profile for smooth acceleration/deceleration
    private TrapezoidProfile.State targetState; // Target state for the trapezoidal profile (rotations, RPM)
    private double maxVelocityRPM; // Mechanism-side max velocity (RPM)
    private double maxAccelerationRPMPerSec; // Mechanism-side max acceleration (RPM/s)
    private double targetVelocityRPM = 0.0; // Target velocity for velocity mode (RPM, mechanism-side)
    private double profiledVelocityRPM = 0.0; // Profiled velocity for velocity mode (RPM, mechanism-side)

    // Noise settings for simulation
    private double positionNoiseStdDev = 0.01; // Standard deviation for position noise (rotations)
    private double velocityNoiseStdDev = 0.05; // Standard deviation for velocity noise (RPM)

    private SIM_MotionProfile motionProfile;

    /**
     * Constructor for MotorIOSim that accepts a SIM_MotionProfile to configure the motor.
     * Initializes the motor simulation and sets up the control systems.
     *
     * @param motor The motor model to simulate (DCMotor).
     * @param momentOfInertia The moment of inertia for the system (kg*m^2).
     * @param gearRatio The gear ratio for the motor system.
     * @param wheelRadiusM The radius of the wheel attached to the motor (meters).
     * @param motionProfile The motion profile containing PID/Feedforward values and constraints.
     * @param noise A boolean flag indicating if noise should be applied to the simulation.
     */
    public MotorIOSim(
        DCMotor motor,
        double momentOfInertia,
        double gearRatio,
        double wheelRadiusM,
        SIM_MotionProfile motionProfile,
        boolean noise
    ) {
        this.gearRatio = gearRatio;
        this.wheelRadiusM = wheelRadiusM;
        this.motionProfile = motionProfile;

        // Initialize the motor simulation based on the motor type and system characteristics
        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, momentOfInertia, gearRatio),
            motor
        );

        // Initialize PID controllers (position in rotations, velocity in RPM)
        this.pidControllerPos = new PIDController(motionProfile.kP_Pos, motionProfile.kI_Pos, motionProfile.kD_Pos);
        this.pidControllerVel = new PIDController(motionProfile.kP_Vel / 60.0, motionProfile.kI_Vel / 60.0, motionProfile.kD_Vel / 60.0);

        // Initialize feedforward controllers (kV in volts per RPM, kA in volts per RPM/s)
        // Note: SimpleMotorFeedforward is deprecated in WPILib 2025; this implementation uses a custom discrete-time feedforward
        this.feedforwardPos = new SimpleMotorFeedforward(motionProfile.kS_Pos, motionProfile.kV_Pos / 60.0, motionProfile.kA_Pos / 60.0);
        this.feedforwardVel = new SimpleMotorFeedforward(motionProfile.kS_Vel, motionProfile.kV_Vel / 60.0, motionProfile.kA_Vel / 60.0);

        // Initialize motion profile constraints (mechanism-side RPM and RPM/s)
        this.maxVelocityRPM = motionProfile.cruiseRPM;
        this.maxAccelerationRPMPerSec = motionProfile.accelRPMPerSec;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocityRPM / 60.0 * TWO_PI, maxAccelerationRPMPerSec / 60.0 * TWO_PI)
        );

        // Set initial target state (mechanism-side rotations, RPM)
        targetState = new TrapezoidProfile.State(motorSim.getAngularPositionRad() / TWO_PI, 0.0);

        // Apply noise if enabled
        if (!noise) {
            positionNoiseStdDev = 0.0;
            velocityNoiseStdDev = 0.0;
        }
    }

    @Override
    public void set_Follower( int id, boolean isBrake, boolean invertedFromLeader){};

    /**
     * Custom discrete-time feedforward calculation to replace SimpleMotorFeedforward.calculate.
     * Computes voltage based on current and next velocity setpoints.
     *
     * @param ff              Feedforward controller (SimpleMotorFeedforward)
     * @param currentVelocityRPM Current velocity (mechanism-side RPM)
     * @param nextVelocityRPM    Next velocity setpoint (mechanism-side RPM)
     * @return Computed feedforward voltage
     */
    private double calculateFeedforward(SimpleMotorFeedforward ff, double currentVelocityRPM, double nextVelocityRPM) {
        double ks = ff.getKs(); // Static gain (volts)
        double kv = ff.getKv() * 60.0; // Velocity gain (volts per RPM)
        double ka = ff.getKa() * 60.0; // Acceleration gain (volts per RPM/s)

        // Handle zero-crossing for static gain
        double staticGain = Math.abs(currentVelocityRPM) < VELOCITY_THRESHOLD ? 0.0 : ks * Math.signum(currentVelocityRPM);

        // Discrete-time feedforward: u = kS * sign(v_current) + kV * v_next + kA * (v_next - v_current) / dt
        if (ka < 1e-9) {
            return staticGain + kv * nextVelocityRPM;
        } else {
            double A = -kv / ka;
            double B = 1.0 / ka;
            double A_d = Math.exp(A * DT);
            double B_d = A > -1e-9 ? B * DT : 1.0 / A * (A_d - 1.0) * B;
            return staticGain + 1.0 / B_d * (nextVelocityRPM - A_d * currentVelocityRPM);
        }
    }

    /*
     * Simplified feedforward calculation (alternative, uncomment if kA is negligible)
     * Computes voltage based on next velocity setpoint, ignoring acceleration term.
     *
     * @param ff              Feedforward controller (SimpleMotorFeedforward)
     * @param currentVelocityRPM Current velocity (mechanism-side RPM)
     * @param nextVelocityRPM    Next velocity setpoint (mechanism-side RPM)
     * @return Computed feedforward voltage
     *
    private double calculateFeedforward(SimpleMotorFeedforward ff, double currentVelocityRPM, double nextVelocityRPM) {
        double ks = ff.getKs(); // Static gain (volts)
        double kv = ff.getKv() * 60.0; // Velocity gain (volts per RPM)

        // Handle zero-crossing for static gain
        double staticGain = Math.abs(currentVelocityRPM) < VELOCITY_THRESHOLD ? 0.0 : ks * Math.signum(currentVelocityRPM);

        // Simplified feedforward: u = kS * sign(v_current) + kV * v_next
        return staticGain + kv * nextVelocityRPM;
    }
    */

    /**
     * Updates the motor inputs and calculates the new motor state based on control and motion profile.
     *
     * @param inputs The current inputs for the motor system, including position, velocity, and sensor data.
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // Get current mechanism-side state from simulation
        double positionRotations = motorSim.getAngularPositionRad() / TWO_PI; // Mechanism-side angle (rotations)
        double velocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / TWO_PI; // Mechanism-side velocity (RPM)

        if (isClosedLoop) {
            // Calculate control voltages
            double ffVolts;
            double pidVolts;

            if (isVelocityControl) {
                double maxDelta = maxAccelerationRPMPerSec * DT;
                double commandedVel = MathUtil.clamp(targetVelocityRPM, -maxVelocityRPM, maxVelocityRPM);
                double desiredDelta = commandedVel - profiledVelocityRPM;
                double delta = MathUtil.clamp(desiredDelta, -maxDelta, maxDelta);
                profiledVelocityRPM += delta;

                // Use custom discrete-time feedforward
                ffVolts = calculateFeedforward(feedforwardVel, velocityRPM, profiledVelocityRPM);
                pidVolts = pidControllerVel.calculate(velocityRPM, profiledVelocityRPM);
            } else {
                TrapezoidProfile.State currentState = new TrapezoidProfile.State(
                    positionRotations * TWO_PI, // Convert to rad for WPILib
                    velocityRPM / 60.0 * TWO_PI // Convert to rad/s for WPILib
                );
                TrapezoidProfile.State nextState = profile.calculate(DT, currentState, targetState);

                // Convert next velocity from rad/s to RPM for feedforward
                double nextVelocityRPM = nextState.velocity * 60.0 / TWO_PI;
                // Use custom discrete-time feedforward
                ffVolts = calculateFeedforward(feedforwardPos, velocityRPM, nextVelocityRPM);
                pidVolts = pidControllerPos.calculate(positionRotations, nextState.position / TWO_PI);
            }

            // Clamp and set applied voltage for closed-loop
            appliedVolts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);

            Logger.recordOutput("PID", pidVolts);
            Logger.recordOutput("ff", ffVolts);
        }

        // Apply voltage to simulation and update state
        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(DT);

        // Populate motor inputs with noise
        double simPositionRotations = motorSim.getAngularPositionRad() / TWO_PI;
        double simVelocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / TWO_PI;

        inputs.motorPositionRot = (simPositionRotations * gearRatio) + (Math.random() - 0.5) * 2 * positionNoiseStdDev;
        inputs.motorVelocityRPM = (simVelocityRPM * gearRatio) + (Math.random() - 0.5) * 2 * velocityNoiseStdDev;

        inputs.mechanismPositionRot = inputs.motorPositionRot / gearRatio;
        inputs.mechanismVelocityRPM = inputs.motorVelocityRPM / gearRatio;

        inputs.tipSpeedMps = (inputs.mechanismVelocityRPM / 60.0 * TWO_PI * wheelRadiusM);

        inputs.appliedVolts = appliedVolts;
        inputs.amps = motorSim.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; // Simulation doesn't provide temperature
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        isClosedLoop = false; // Disable closed-loop control
        isVelocityControl = false;
    }

    @Override
    public void setPercent(double percent) {
        setVoltage(percent * 12.0);
    }

    @Override
    public void setTargetMotorPosition(double motorRotations) {
        // Convert motor-side rotations to mechanism-side rotations
        double positionRotations = motorRotations / gearRatio;
        targetState = new TrapezoidProfile.State(positionRotations * TWO_PI, 0.0); // Convert to rad for WPILib
        if (!isClosedLoop || isVelocityControl) {
            pidControllerPos.reset();
        }
        isVelocityControl = false;
        isClosedLoop = true; // Enable closed-loop control
    }

    @Override
    public void setCurrentMotorPosition(double motorRotations) {
        // Convert motor-side rotations to mechanism-side radians for sim
        double positionRadians = (motorRotations / gearRatio) * TWO_PI;
        motorSim.setState(positionRadians, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void setTargetMechanismPosition(double mechanismRotations) {
        targetState = new TrapezoidProfile.State(mechanismRotations * TWO_PI, 0.0); // Convert to rad for WPILib
        if (!isClosedLoop || isVelocityControl) {
            pidControllerPos.reset();
        }
        isVelocityControl = false;
        isClosedLoop = true; // Enable closed-loop control
    }

    @Override
    public void setCurrentMechanismPosition(double mechanismRotations) {
        // Convert mechanism-side rotations to radians for sim
        double positionRadians = mechanismRotations * TWO_PI;
        motorSim.setState(positionRadians, motorSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void setTargetMotorVelocity(double motorRPM) {
        // Convert motor-side RPM to mechanism-side RPM
        double velocityRPM = motorRPM / gearRatio;
        targetVelocityRPM = velocityRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / TWO_PI;
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true; // Enable closed-loop control
    }

    @Override
    public void setTargetMechanismVelocity(double mechanismRPM) {
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / TWO_PI;
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true; // Enable closed-loop control
    }

    @Override
    public void setTargetTipSpeed(double mps) {
        // Convert linear speed to mechanism-side RPM
        double mechanismRPM = mps / (wheelRadiusM * TWO_PI) * 60.0;
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = motorSim.getAngularVelocityRadPerSec() * 60.0 / TWO_PI;
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true; // Enable closed-loop control
    }

    @Override
    public void stop() {
        setVoltage(0.0);
        isClosedLoop = false; // Disable closed-loop control
        isVelocityControl = false;
    }

    @Override
    public void setTunerConstants(double p, double i, double d, double ff,
                                  double iZone, double iMaxAccum,
                                  double minOutput, double maxOutput, int slot) {
        if (isVelocityControl) {
            pidControllerVel.setP(p / 60.0); // Scale for RPM
            pidControllerVel.setI(i / 60.0);
            pidControllerVel.setD(d / 60.0);
            if (ff != 0.0) {
                // Note: SimpleMotorFeedforward is deprecated in WPILib 2025; this implementation uses a custom discrete-time feedforward
                feedforwardVel = new SimpleMotorFeedforward(ff, motionProfile.kV_Vel / 60.0, motionProfile.kA_Vel / 60.0);
            }
        } else {
            pidControllerPos.setP(p);
            pidControllerPos.setI(i);
            pidControllerPos.setD(d);
            if (ff != 0.0) {
                // Note: SimpleMotorFeedforward is deprecated in WPILib 2025; this implementation uses a custom discrete-time feedforward
                feedforwardPos = new SimpleMotorFeedforward(ff, motionProfile.kV_Pos / 60.0, motionProfile.kA_Pos / 60.0);
            }
        }
        // Note: iZone, iMaxAccum, minOutput, maxOutput, and slot are not supported
    }

    @Override
    public void setMotionConstraints(double maxVel, double maxAccel, double allowedError, int slot) {
        // Expect maxVel in RPM and maxAccel in RPM/s (mechanism-side)
        this.maxVelocityRPM = maxVel;
        this.maxAccelerationRPMPerSec = maxAccel;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVel / 60.0 * TWO_PI, maxAccel / 60.0 * TWO_PI)
        );
        this.pidControllerPos.setTolerance(allowedError);
        this.pidControllerVel.setTolerance(allowedError);
    }

    @Override
    public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
        // Simulation does not support current limits; no-op
    }

}