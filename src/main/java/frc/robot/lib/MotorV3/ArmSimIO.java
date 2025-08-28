package frc.robot.lib.MotorV3;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.Motor.MotionProfileHelpers.SIM_MotionProfile;

/**
 * Simulation for an arm motor using SingleJointedArmSim, supporting position and velocity control.
 * All motor inputs are in rotations or RPM (motor-side). Simulation interactions are in rotations or RPM (mechanism-side).
 * minAngle and maxAngle are mechanism-side rotations.
 */
public class ArmSimIO implements MotorIO {

  private static final double TWO_PI = 2.0 * Math.PI; // Constant for 2π
  private static final double DT = 0.02; // 20ms update period

  // Simulation components
  private final SingleJointedArmSim pivot; // Arm simulation
  private final double gearRatio; // Motor rotations per mechanism rotation
  private final double armLengthMeters; // Arm length in meters
  private double appliedVolts = 0.0; // Applied voltage
  private boolean isVelocityControl = false; // Control mode flag (velocity vs position)
  private boolean isClosedLoop = false; // Flag to enable/disable closed-loop control (PID + FF + profile)

  // Motion control components
  private PIDController pidControllerPos; // Position PID (rotations)
  private PIDController pidControllerVel; // Velocity PID (RPM)
  private ArmFeedforward feedforwardPos; // Position feedforward (volts per rad/s)
  private ArmFeedforward feedforwardVel; // Velocity feedforward (volts per rad/s)
  private TrapezoidProfile profile; // Trapezoidal motion profile
  private TrapezoidProfile.State targetState; // Target state (rotations, RPM)
  private double maxVelocityRPM; // Mechanism-side max velocity (RPM)
  private double maxAccelerationRPMPerSec; // Mechanism-side max acceleration (RPM/s)
  private double targetVelocityRPM = 0.0; // Target velocity for velocity mode (RPM)
  private double profiledVelocityRPM = 0.0; // Profiled velocity for velocity mode (RPM)
  private final SIM_MotionProfile motionProfile; // Store for reference (e.g., kS_Pos)

  /**
   * Constructor for ArmSimIO.
   *
   * @param motor            Motor model (DCMotor)
   * @param gearRatio        Motor rotations per mechanism rotation
   * @param armLengthMeters  Length of the arm (meters)
   * @param momentOfInertia  Moment of inertia (kg·m²)
   * @param minAngle         Minimum mechanism angle (rotations, mechanism-side)
   * @param maxAngle         Maximum mechanism angle (rotations, mechanism-side)
   * @param startingAngle    Starting mechanism angle (rotations, mechanism-side)
   * @param motionProfile    Motion profile with PID/feedforward gains and constraints
   */
  public ArmSimIO(
      DCMotor motor,
      double gearRatio,
      double armLengthMeters,
      double momentOfInertia,
      double minAngle,
      double maxAngle,
      double startingAngle,
      SIM_MotionProfile motionProfile
  ) {
      this.gearRatio = gearRatio;
      this.armLengthMeters = armLengthMeters;
      this.motionProfile = motionProfile;

      // Initialize arm simulation with mechanism-side angles in radians
      this.pivot = new SingleJointedArmSim(
          motor,
          gearRatio,
          momentOfInertia,
          armLengthMeters,
          Units.rotationsToRadians(minAngle), // Mechanism-side min angle (radians)
          Units.rotationsToRadians(maxAngle), // Mechanism-side max angle (radians)
          true, // Simulate gravity
          Units.rotationsToRadians(startingAngle) // Mechanism-side starting angle (radians)
      );

      // Initialize PID controllers (position in rotations, velocity in RPM)
      this.pidControllerPos = new PIDController(motionProfile.kP_Pos, motionProfile.kI_Pos, motionProfile.kD_Pos);
      this.pidControllerVel = new PIDController(motionProfile.kP_Vel / 60.0, motionProfile.kI_Vel / 60.0, motionProfile.kD_Vel / 60.0);

      // Initialize feedforward controllers (kV in volts per rad/s, kA in volts per rad/s², kG in volts)
      this.feedforwardPos = new ArmFeedforward(motionProfile.kS_Pos,  motionProfile.kG, motionProfile.kV_Pos * 60.0 / TWO_PI, motionProfile.kA_Pos * 60.0 / TWO_PI);
      this.feedforwardVel = new ArmFeedforward(motionProfile.kS_Vel, motionProfile.kG, motionProfile.kV_Vel * 60.0 / TWO_PI, motionProfile.kA_Vel * 60.0 / TWO_PI);

      // Initialize motion profile constraints (mechanism-side RPM and RPM/s)
      this.maxVelocityRPM = motionProfile.cruiseRPM;
      this.maxAccelerationRPMPerSec = motionProfile.accelRPMPerSec;
      this.profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              maxVelocityRPM / 60.0 * TWO_PI, // Convert RPM to rad/s
              maxAccelerationRPMPerSec / 60.0 * TWO_PI // Convert RPM/s to rad/s²
          )
      );

      // Set initial target state (mechanism-side rotations, RPM)
      targetState = new TrapezoidProfile.State(Units.rotationsToRadians(startingAngle), 0.0);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
      // Get current mechanism-side state from simulation
      double positionRotations = pivot.getAngleRads() / TWO_PI; // Mechanism-side angle (rotations)
      double velocityRPM = pivot.getVelocityRadPerSec() * 60.0 / TWO_PI; // Mechanism-side velocity (RPM)

      if (isClosedLoop) {
          // Calculate control voltages
          double ffVolts;
          double pidVolts;
          double desiredAccel;

          if (isVelocityControl) {
              double maxDelta = maxAccelerationRPMPerSec * DT;
              double commandedVel = MathUtil.clamp(targetVelocityRPM, -maxVelocityRPM, maxVelocityRPM);
              double desiredDelta = commandedVel - profiledVelocityRPM;
              double delta = MathUtil.clamp(desiredDelta, -maxDelta, maxDelta);
              profiledVelocityRPM += delta;

              desiredAccel = delta / DT; // Acceleration in RPM/s
              ffVolts = feedforwardVel.calculate(profiledVelocityRPM / 60.0 * TWO_PI, desiredAccel / 60.0 * TWO_PI);
              pidVolts = pidControllerVel.calculate(velocityRPM, profiledVelocityRPM);
          } else {
              TrapezoidProfile.State currentState = new TrapezoidProfile.State(
                  positionRotations * TWO_PI, // Convert to rad for WPILib
                  velocityRPM / 60.0 * TWO_PI // Convert to rad/s for WPILib
              );
              TrapezoidProfile.State nextState = profile.calculate(DT, currentState, targetState);

              desiredAccel = (nextState.velocity * 60.0 / TWO_PI - velocityRPM) / DT; // Convert rad/s to RPM/s
              ffVolts = feedforwardPos.calculate(nextState.velocity, desiredAccel / 60.0 * TWO_PI);
              pidVolts = pidControllerPos.calculate(positionRotations, nextState.position / TWO_PI);
          }

          // Clamp and set applied voltage for closed-loop
          appliedVolts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);

          Logger.recordOutput("PID", pidVolts);
          Logger.recordOutput("ff", ffVolts);
      }

      // Apply voltage to simulation and update state
      pivot.setInputVoltage(appliedVolts);
      pivot.update(DT);

      // Populate motor inputs
      inputs.motorPositionRot = positionRotations * gearRatio; // Mechanism rotations to motor rotations
      inputs.motorVelocityRPM = velocityRPM * gearRatio; // Mechanism RPM to motor RPM
      inputs.mechanismPositionRot = positionRotations; // Mechanism rotations
      inputs.mechanismVelocityRPM = velocityRPM; // Mechanism RPM
      inputs.tipSpeedMps = velocityRPM / 60.0 * TWO_PI * armLengthMeters; // Mechanism RPM to m/s
      inputs.appliedVolts = appliedVolts;
      inputs.amps = pivot.getCurrentDrawAmps();
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
      // Convert motor-side rotations to mechanism-side radians
      pivot.setState(motorRotations * TWO_PI / gearRatio, pivot.getVelocityRadPerSec());
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
      // Convert mechanism-side rotations to radians
      pivot.setState(mechanismRotations * TWO_PI, pivot.getVelocityRadPerSec());
  }

  @Override
  public void setTargetMotorVelocity(double motorRPM) {
      // Convert motor-side RPM to mechanism-side RPM
      double velocityRPM = motorRPM / gearRatio;
      targetVelocityRPM = velocityRPM;
      if (!isClosedLoop || !isVelocityControl) {
          profiledVelocityRPM = pivot.getVelocityRadPerSec() * 60.0 / TWO_PI;
          pidControllerVel.reset();
      }
      isVelocityControl = true;
      isClosedLoop = true; // Enable closed-loop control
  }

  @Override
  public void setTargetMechanismVelocity(double mechanismRPM) {
      targetVelocityRPM = mechanismRPM;
      if (!isClosedLoop || !isVelocityControl) {
          profiledVelocityRPM = pivot.getVelocityRadPerSec() * 60.0 / TWO_PI;
          pidControllerVel.reset();
      }
      isVelocityControl = true;
      isClosedLoop = true; // Enable closed-loop control
  }

  @Override
  public void setTargetTipSpeed(double mps) {
      // Convert linear speed to mechanism-side RPM
      double mechanismRPM = mps / armLengthMeters * 60.0 / TWO_PI;
      targetVelocityRPM = mechanismRPM;
      if (!isClosedLoop || !isVelocityControl) {
          profiledVelocityRPM = pivot.getVelocityRadPerSec() * 60.0 / TWO_PI;
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
              feedforwardVel = new ArmFeedforward(ff, motionProfile.kV_Vel * 60.0 / TWO_PI, motionProfile.kA_Vel * 60.0 / TWO_PI, ff);
          }
      } else {
          pidControllerPos.setP(p);
          pidControllerPos.setI(i);
          pidControllerPos.setD(d);
          if (ff != 0.0) {
              feedforwardPos = new ArmFeedforward(ff, motionProfile.kV_Pos * 60.0 / TWO_PI, motionProfile.kA_Pos * 60.0 / TWO_PI, ff);
          }
      }
      // Note: ff, iZone, iMaxAccum, minOutput, maxOutput, and slot are not supported
  }

  @Override
  public void setMotionConstraints(double maxVel, double maxAccel, double allowedError, int slot) {
      // Expect maxVel in RPM and maxAccel in RPM/s (mechanism-side)
      this.maxVelocityRPM = maxVel;
      this.maxAccelerationRPMPerSec = maxAccel;
      this.profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              maxVel / 60.0 * TWO_PI, // Convert RPM to rad/s
              maxAccel / 60.0 * TWO_PI // Convert RPM/s to rad/s²
          )
      );
      this.pidControllerPos.setTolerance(allowedError);
      this.pidControllerVel.setTolerance(allowedError);
  }

}