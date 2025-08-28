package frc.robot.lib.MotorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.Motor.MotionProfileHelpers.SIM_MotionProfile;
import org.littletonrobotics.junction.Logger;

public class FlyWheelSimIO implements MotorIO {
    private static final double TWO_PI = 2.0 * Math.PI; // Constant for 2π, used for radian conversions
    private static final double DT = 0.02; // 20ms update period
    private static final double VELOCITY_THRESHOLD = 1e-6; // Small velocity threshold for zero-crossing (RPM)

    // Simulation components
    private final FlywheelSim gripperSim; // Flywheel simulation
    private final double gearRatio; // Motor rotations per mechanism rotation
    private final double wheelRadius; // Wheel radius in meters
    private double appliedVolts = 0.0; // Applied voltage
    private boolean isVelocityControl = false; // Control mode flag (velocity vs position)
    private boolean isClosedLoop = false; // Flag to enable/disable closed-loop control
    private double currentPositionRot = 0.0; // Track position for simulation (motor-side rotations)

    // Motion control components
    private PIDController pidControllerPos; // Position PID (rotations)
    private PIDController pidControllerVel; // Velocity PID (RPM)
    private SimpleMotorFeedforward feedforwardPos; // Position feedforward (volts per RPM)
    private SimpleMotorFeedforward feedforwardVel; // Velocity feedforward (volts per RPM)
    private TrapezoidProfile profile; // Trapezoidal motion profile
    private TrapezoidProfile.State targetState; // Target state (mechanism-side rotations, RPM)
    private double maxVelocityRPM; // Mechanism-side max velocity (RPM)
    private double maxAccelerationRPMPerSec; // Mechanism-side max acceleration (RPM/s)
    private double targetVelocityRPM = 0.0; // Target velocity for velocity mode (RPM, mechanism-side)
    private double profiledVelocityRPM = 0.0; // Profiled velocity for velocity mode (RPM, mechanism-side)
    private final SIM_MotionProfile motionProfile; // Store for PID/feedforward gains and constraints

    /**
     * Constructor for FlyWheelSimIO.
     *
     * @param motor            Motor model (DCMotor)
     * @param gearRatio        Motor rotations per mechanism rotation
     * @param wheelRadius      Wheel radius in meters
     * @param momentOfInertia  Moment of inertia (kg·m²)
     * @param motionProfile    Motion profile with PID/feedforward gains and constraints
     */
    public FlyWheelSimIO(
        DCMotor motor,
        double gearRatio,
        double wheelRadius,
        double momentOfInertia,
        SIM_MotionProfile motionProfile
    ) {
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.motionProfile = motionProfile;

        // Initialize flywheel simulation
        this.gripperSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, momentOfInertia, gearRatio),
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
            new TrapezoidProfile.Constraints(motionProfile.cruiseRPM / 60.0 * TWO_PI, motionProfile.accelRPMPerSec / 60.0 * TWO_PI)
        );

        // Set initial target state (mechanism-side rotations, RPM)
        targetState = new TrapezoidProfile.State(0.0, 0.0);
    }

    /**
     * Custom discrete-time feedforward calculation to replace SimpleMotorFeedforward.calculate.
     * Computes voltage based on current and next velocity setpoints.
     *
     * @param ff       Feedforward controller (SimpleMotorFeedforward)
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

        // Discrete-time feedforward: u = kS * sign(v) + kV * v_next + kA * (v_next - v_current) / dt
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

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // Get current mechanism-side state from simulation
        double mechanismRPM = gripperSim.getAngularVelocityRPM(); // Mechanism-side RPM
        double motorRPM = mechanismRPM * gearRatio; // Motor-side RPM
        double mechanismRotations = currentPositionRot / gearRatio; // Mechanism-side rotations
        double mechanismSpeedMps = mechanismRPM / 60.0 * TWO_PI * wheelRadius; // Mechanism speed (m/s)

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
                ffVolts = calculateFeedforward(feedforwardVel, mechanismRPM, profiledVelocityRPM);
                pidVolts = pidControllerVel.calculate(mechanismRPM, profiledVelocityRPM);
            } else {
                TrapezoidProfile.State currentState = new TrapezoidProfile.State(
                    mechanismRotations * TWO_PI, // Convert to rad for WPILib
                    mechanismRPM / 60.0 * TWO_PI // Convert to rad/s for WPILib
                );
                TrapezoidProfile.State nextState = profile.calculate(DT, currentState, targetState);

                // Convert next velocity from rad/s to RPM for feedforward
                double nextVelocityRPM = nextState.velocity * 60.0 / TWO_PI;
                // Use custom discrete-time feedforward
                ffVolts = calculateFeedforward(feedforwardPos, mechanismRPM, nextVelocityRPM);
                pidVolts = pidControllerPos.calculate(mechanismRotations, nextState.position / TWO_PI);
            }

            // Clamp and set applied voltage for closed-loop
            appliedVolts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);

            Logger.recordOutput("PID", pidVolts);
            Logger.recordOutput("FF", ffVolts);
        }

        // Apply voltage to simulation and update state
        gripperSim.setInputVoltage(appliedVolts);
        gripperSim.update(DT);

        // Update position based on velocity (integrate velocity over time)
        double mechanismRotPerSec = gripperSim.getAngularVelocityRPM() / 60.0; // Mechanism-side rotations/sec
        double motorRotPerSec = mechanismRotPerSec * gearRatio; // Motor-side rotations/sec
        currentPositionRot += motorRotPerSec * DT; // Update position (motor-side rotations)

        // Populate motor inputs
        inputs.motorPositionRot = currentPositionRot; // Motor-side rotations
        inputs.motorVelocityRPM = motorRPM; // Motor-side RPM
        inputs.mechanismPositionRot = currentPositionRot / gearRatio; // Mechanism-side rotations
        inputs.mechanismVelocityRPM = mechanismRPM; // Mechanism-side RPM
        inputs.tipSpeedMps = mechanismSpeedMps; // Mechanism speed in m/s
        inputs.appliedVolts = appliedVolts;
        inputs.amps = gripperSim.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; // Simulation doesn't provide temperature
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        isClosedLoop = false;
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
        isClosedLoop = true;
    }

    @Override
    public void setCurrentMotorPosition(double motorRotations) {
        currentPositionRot = motorRotations; // Directly set motor-side position
    }

    @Override
    public void setTargetMechanismPosition(double mechanismRotations) {
        targetState = new TrapezoidProfile.State(mechanismRotations * TWO_PI, 0.0); // Convert to rad for WPILib
        if (!isClosedLoop || isVelocityControl) {
            pidControllerPos.reset();
        }
        isVelocityControl = false;
        isClosedLoop = true;
    }

    @Override
    public void setCurrentMechanismPosition(double mechanismRotations) {
        // Convert mechanism-side rotations to motor-side rotations
        currentPositionRot = mechanismRotations * gearRatio;
    }

    @Override
    public void setTargetMotorVelocity(double motorRPM) {
        // Convert motor-side RPM to mechanism-side RPM
        double velocityRPM = motorRPM / gearRatio;
        targetVelocityRPM = velocityRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = gripperSim.getAngularVelocityRPM();
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true;
    }

    @Override
    public void setTargetMechanismVelocity(double mechanismRPM) {
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = gripperSim.getAngularVelocityRPM();
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true;
    }

    @Override
    public void setTargetTipSpeed(double mps) {
        // Convert tip speed (m/s) to mechanism-side RPM
        double mechanismRPM = mps / (wheelRadius * TWO_PI) * 60.0;
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = gripperSim.getAngularVelocityRPM();
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true;
    }

    @Override
    public void stop() {
        setVoltage(0.0);
        isClosedLoop = false;
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
}