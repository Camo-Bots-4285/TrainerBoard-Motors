// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.MotorV3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.lib.Motor.MotionProfileHelpers.SIM_MotionProfile;
import org.littletonrobotics.junction.Logger;

public class ElevatorSimIO implements MotorIO {

    private static final double TWO_PI = 2.0 * Math.PI; // Constant for 2π
    private static final double DT = 0.02; // 20ms update period

    private final ElevatorSim elevatorSim;
    private final double gearRatio;
    private final double drumRadiusMeters;
    private final SIM_MotionProfile motionProfile;

    private PIDController pidControllerPos; // Position PID (drum rotations)
    private PIDController pidControllerVel; // Velocity PID (drum RPM)
    private ElevatorFeedforward feedforwardPos; // Position feedforward (volts per m/s)
    private ElevatorFeedforward feedforwardVel; // Velocity feedforward (volts per m/s)

    private TrapezoidProfile profile;
    private TrapezoidProfile.State targetState = new TrapezoidProfile.State(0.0, 0.0); // Meters, m/s for WPILib

    private double maxVelocityRPM; // Mechanism-side max velocity (drum RPM)
    private double maxAccelerationRPMPerSec; // Mechanism-side max acceleration (drum RPM/s)
    private double targetVelocityRPM = 0.0; // Target velocity for velocity mode (drum RPM)
    private double profiledVelocityRPM = 0.0; // Profiled velocity for velocity mode (drum RPM)

    private double appliedVolts = 0.0;
    private boolean isVelocityControl = false;
    private boolean isClosedLoop = false;

    public ElevatorSimIO(
        DCMotor motor,
        double gearRatio,
        double drumRadiusMeters,
        double carriageMassKg,
        double minHeightMeters,
        double maxHeightMeters,
        double startingHeightMeters,
        boolean simulateGravity,
        SIM_MotionProfile motionProfile
    ) {
        this.gearRatio = gearRatio;
        this.drumRadiusMeters = drumRadiusMeters;
        this.motionProfile = motionProfile;

        this.elevatorSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(motor, carriageMassKg, drumRadiusMeters, gearRatio),
            motor,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters
        );

        // Initialize motion profile constraints (interpreted as drum RPM and RPM/s)
        this.maxVelocityRPM = motionProfile.cruiseRPM; // Drum RPM
        this.maxAccelerationRPMPerSec = motionProfile.accelRPMPerSec; // Drum RPM/s

        // Initialize PID controllers (position in drum rotations, velocity in drum RPM)
        pidControllerPos = new PIDController(motionProfile.kP_Pos, motionProfile.kI_Pos, motionProfile.kD_Pos);
        // Scale velocity gains for RPM (meters/s to RPM: * 60 / (2 * π * drumRadiusMeters))
        double velScale = 60.0 / (TWO_PI * drumRadiusMeters);
        pidControllerVel = new PIDController(
            motionProfile.kP_Vel / velScale,
            motionProfile.kI_Vel / velScale,
            motionProfile.kD_Vel / velScale
        );

        // Initialize feedforward controllers (kV in volts per m/s, kA in volts per m/s², kG in volts)
        feedforwardPos = new ElevatorFeedforward(
            motionProfile.kS_Pos,
            motionProfile.kG,
            motionProfile.kV_Pos * velScale,
            motionProfile.kA_Pos * velScale
        );
        feedforwardVel = new ElevatorFeedforward(
            motionProfile.kS_Vel,
            motionProfile.kG,
            motionProfile.kV_Vel * velScale,
            motionProfile.kA_Vel * velScale
        );

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityRPM / 60.0 * TWO_PI * drumRadiusMeters, // Convert RPM to m/s
                maxAccelerationRPMPerSec / 60.0 * TWO_PI * drumRadiusMeters // Convert RPM/s to m/s²
            )
        );

        // Set initial target state (mechanism-side drum rotations, RPM)
        targetState = new TrapezoidProfile.State(startingHeightMeters, 0.0); // Meters for WPILib
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        double positionMeters = elevatorSim.getPositionMeters();
        double velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        // Convert to drum rotations and RPM
        double mechanismRotations = positionMeters / (TWO_PI * drumRadiusMeters); // Drum rotations
        double mechanismRPM = velocityMetersPerSec * 60.0 / (TWO_PI * drumRadiusMeters); // Drum RPM

        if (isClosedLoop) {
            double ffVolts;
            double pidVolts;

            if (isVelocityControl) {
                double maxDelta = maxAccelerationRPMPerSec * DT;
                double commandedVel = MathUtil.clamp(targetVelocityRPM, -maxVelocityRPM, maxVelocityRPM);
                double desiredDelta = commandedVel - profiledVelocityRPM;
                double delta = MathUtil.clamp(desiredDelta, -maxDelta, maxDelta);
                profiledVelocityRPM += delta;

                // Convert velocities to m/s for feedforward
                double currentVelocityMps = velocityMetersPerSec; // Current velocity (m/s)
                double nextVelocityMps = profiledVelocityRPM / 60.0 * TWO_PI * drumRadiusMeters; // Next velocity (m/s)

                // Use calculateWithVelocities for discrete control
                ffVolts = feedforwardVel.calculateWithVelocities(currentVelocityMps, nextVelocityMps);
                pidVolts = pidControllerVel.calculate(mechanismRPM, profiledVelocityRPM);
            } else {
                TrapezoidProfile.State currentState = new TrapezoidProfile.State(positionMeters, velocityMetersPerSec);
                TrapezoidProfile.State nextState = profile.calculate(DT, currentState, targetState);

                // Convert velocities to m/s for feedforward
                double currentVelocityMps = velocityMetersPerSec; // Current velocity (m/s)
                double nextVelocityMps = nextState.velocity; // Next velocity (m/s)

                // Use calculateWithVelocities for discrete control
                ffVolts = feedforwardPos.calculateWithVelocities(currentVelocityMps, nextVelocityMps);
                pidVolts = pidControllerPos.calculate(mechanismRotations, nextState.position / (TWO_PI * drumRadiusMeters));
            }

            appliedVolts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);

            Logger.recordOutput("PID", pidVolts);
            Logger.recordOutput("FF", ffVolts);
        }

        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(DT);

        // Motor rotations and velocity
        inputs.motorPositionRot = (positionMeters / (TWO_PI * drumRadiusMeters)) * gearRatio;
        inputs.motorVelocityRPM = (velocityMetersPerSec * 60.0 / (TWO_PI * drumRadiusMeters)) * gearRatio;

        // Mechanism position and velocity (drum rotations and RPM)
        inputs.mechanismPositionRot = mechanismRotations;
        inputs.mechanismVelocityRPM = mechanismRPM;

        inputs.tipSpeedMps = 0.0; // Not used in this sim
        inputs.appliedVolts = appliedVolts;
        inputs.amps = elevatorSim.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; // Not modeled
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
        // Convert motor rotations to mechanism-side drum rotations, then to meters
        double mechanismRotations = motorRotations / gearRatio;
        double meters = mechanismRotations * TWO_PI * drumRadiusMeters;
        targetState = new TrapezoidProfile.State(meters, 0.0);
        if (!isClosedLoop || isVelocityControl) {
            pidControllerPos.reset();
        }
        isVelocityControl = false;
        isClosedLoop = true;
    }

    @Override
    public void setCurrentMotorPosition(double motorRotations) {
        // Convert motor rotations to meters
        double meters = (motorRotations / gearRatio) * TWO_PI * drumRadiusMeters;
        elevatorSim.setState(meters, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void setTargetMechanismPosition(double mechanismRotations) {
        // Convert mechanism-side drum rotations to meters
        double meters = mechanismRotations * TWO_PI * drumRadiusMeters;
        targetState = new TrapezoidProfile.State(meters, 0.0);
        if (!isClosedLoop || isVelocityControl) {
            pidControllerPos.reset();
        }
        isVelocityControl = false;
        isClosedLoop = true;
    }

    @Override
    public void setCurrentMechanismPosition(double mechanismRotations) {
        // Convert mechanism-side drum rotations to meters
        double meters = mechanismRotations * TWO_PI * drumRadiusMeters;
        elevatorSim.setState(meters, elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void setTargetMotorVelocity(double motorRPM) {
        // Convert motor RPM to mechanism-side drum RPM
        double mechanismRPM = motorRPM / gearRatio;
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = elevatorSim.getVelocityMetersPerSecond() * 60.0 / (TWO_PI * drumRadiusMeters);
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true;
    }

    @Override
    public void setTargetMechanismVelocity(double mechanismRPM) {
        targetVelocityRPM = mechanismRPM;
        if (!isClosedLoop || !isVelocityControl) {
            profiledVelocityRPM = elevatorSim.getVelocityMetersPerSecond() * 60.0 / (TWO_PI * drumRadiusMeters);
            pidControllerVel.reset();
        }
        isVelocityControl = true;
        isClosedLoop = true;
    }

    @Override
    public void setTargetTipSpeed(double mps) {
        Logger.recordOutput("ElevatorSimIO/Warning", "Tip speed control not supported for elevator");
    }

    @Override
    public void stop() {
        setVoltage(0.0);
        isClosedLoop = false;
        isVelocityControl = false;
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        // ElevatorSim does not support brake mode simulation; no-op
    }

    @Override
    public void setTunerConstants(double p, double i, double d, double ff,
                                  double iZone, double iMaxAccum,
                                  double minOutput, double maxOutput, int slot) {
        double velScale = 60.0 / (TWO_PI * drumRadiusMeters); // m/s to RPM
        if (isVelocityControl) {
            pidControllerVel.setP(p / velScale);
            pidControllerVel.setI(i / velScale);
            pidControllerVel.setD(d / velScale);
            if (ff != 0.0) {
                feedforwardVel = new ElevatorFeedforward(ff, motionProfile.kG, motionProfile.kV_Vel * velScale, motionProfile.kA_Vel * velScale);
            }
        } else {
            pidControllerPos.setP(p);
            pidControllerPos.setI(i);
            pidControllerPos.setD(d);
            if (ff != 0.0) {
                feedforwardPos = new ElevatorFeedforward(ff, motionProfile.kG, motionProfile.kV_Pos * velScale, motionProfile.kA_Pos * velScale);
            }
        }
        // Note: iZone, iMaxAccum, minOutput, maxOutput, and slot are not supported
    }

    @Override
    public void setMotionConstraints(double maxVel, double maxAccel, double allowedError, int slot) {
        // Expect maxVel in drum RPM and maxAccel in drum RPM/s
        this.maxVelocityRPM = maxVel;
        this.maxAccelerationRPMPerSec = maxAccel;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVel / 60.0 * TWO_PI * drumRadiusMeters, // Convert RPM to m/s
                maxAccel / 60.0 * TWO_PI * drumRadiusMeters // Convert RPM/s to m/s²
            )
        );
        this.pidControllerPos.setTolerance(allowedError / (TWO_PI * drumRadiusMeters)); // Convert rotations to meters
        this.pidControllerVel.setTolerance(allowedError * 60.0 / (TWO_PI * drumRadiusMeters)); // Convert RPM to m/s
    }

    @Override
    public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
        // Not implemented in simulation
    }
}