// package frc.robot.lib.MotorV3; // Aligned package with class version

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim; // Added for elevator simulation
// import org.littletonrobotics.junction.Logger;

// @SuppressWarnings("rawtypes")
// public class MotorIOSimV4 implements MotorIO {
//   private static final double SIM_TIME_STEP_SECONDS = 0.02; // Configurable timestep
//   private final LinearSystemSim sim;
//   private final PIDController pid;
//   private final double gearRatio;
//   private final double mechanismLengthMeters;
//   private final double drumRadiusMeters; // Added for elevator simulation
//   private boolean isClosedLoop = false;
//   private double appliedVolts = 0.0;
//   private double currentPositionRot = 0.0;

//   /**
//    * Constructor for MotorIOSim.
//    *
//    * @param sim The LinearSystemSim object (e.g., SingleJointedArmSim, FlywheelSim, DCMotorSim, ElevatorSim).
//    * @param gearRatio The gear ratio from motor to mechanism (must be positive).
//    * @param mechanismLengthMeters The length of the mechanism (e.g., arm length) for tip speed calculation, or 0 if not applicable.
//    * @param drumRadiusMeters The radius of the drum or spool for elevator systems, or 0 if not applicable.
//    */
//   public MotorIOSimV4(LinearSystemSim sim, double gearRatio, double mechanismLengthMeters, double drumRadiusMeters) {
//     if (gearRatio <= 0) {
//       throw new IllegalArgumentException("Gear ratio must be positive");
//     }
//     if (mechanismLengthMeters < 0) {
//       throw new IllegalArgumentException("Mechanism length cannot be negative");
//     }
//     if (drumRadiusMeters < 0) {
//       throw new IllegalArgumentException("Drum radius cannot be negative");
//     }
//     this.sim = sim;
//     this.gearRatio = gearRatio;
//     this.mechanismLengthMeters = mechanismLengthMeters;
//     this.drumRadiusMeters = drumRadiusMeters; // Initialize new parameter
//     this.pid = new PIDController(0, 0, 0);
//   }

//   @Override
//   public void updateInputs(MotorIOInputs inputs) {
//     // Update simulation with applied voltage
//     if (isClosedLoop) {
//       double feedbackValue = getFeedbackValue();
//       appliedVolts = pid.calculate(feedbackValue);
//     }
//     // Use setInput for compatibility with all LinearSystemSim subclasses
//     sim.setInput(appliedVolts);
//     sim.update(SIM_TIME_STEP_SECONDS);

//     // Get position and velocity based on simulation type
//     double motorVelocityRPM = 0.0;
//     double amps = 0.0;
//     if (sim instanceof SingleJointedArmSim armSim) {
//       currentPositionRot = armSim.getAngleRads() / (2 * Math.PI);
//       motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
//       amps = armSim.getCurrentDrawAmps();
//     } else if (sim instanceof FlywheelSim flywheelSim) {
//       double velocityRadPerSec = flywheelSim.getOutput(0); // Output in rad/s
//       currentPositionRot += Units.radiansToRotations(velocityRadPerSec * SIM_TIME_STEP_SECONDS);
//       motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
//       amps = flywheelSim.getCurrentDrawAmps();
//     } else if (sim instanceof DCMotorSim dcMotorSim) {
//       currentPositionRot = dcMotorSim.getAngularPositionRad() / (2 * Math.PI);
//       motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(dcMotorSim.getAngularVelocityRadPerSec());
//       amps = dcMotorSim.getCurrentDrawAmps();
//     } else if (sim instanceof ElevatorSim elevatorSim) { // Added ElevatorSim support
//       double elevatorPositionMeters = elevatorSim.getPositionMeters();
//       double elevatorVelocityMps = elevatorSim.getVelocityMetersPerSecond();
//       // Convert linear position to motor rotations: meters / (2 * π * drum_radius) * gear_ratio
//       currentPositionRot = (elevatorPositionMeters / (2 * Math.PI * drumRadiusMeters)) * gearRatio;
//       // Convert linear velocity to motor RPM: (m/s / (2 * π * drum_radius)) * gear_ratio * 60
//       motorVelocityRPM = (elevatorVelocityMps / (2 * Math.PI * drumRadiusMeters)) * gearRatio * 60.0;
//       amps = elevatorSim.getCurrentDrawAmps();
//     } else {
//       Logger.recordOutput("MotorIOSim/Warning", "Unrecognized simulation type: " + sim.getClass().getSimpleName());
//       double velocityRadPerSec = sim.getOutput(0); // Assume rad/s
//       currentPositionRot += Units.radiansToRotations(velocityRadPerSec * SIM_TIME_STEP_SECONDS);
//       motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
//     }

//     // Populate inputs
//     inputs.motorPositionRot = currentPositionRot;
//     inputs.motorVelocityRPM = motorVelocityRPM;
//     inputs.mechanismPositionRot = currentPositionRot / gearRatio;
//     inputs.mechanismVelocityRPM = motorVelocityRPM / gearRatio;
//     inputs.tipSpeedMps = (mechanismLengthMeters > 0 && !(sim instanceof ElevatorSim)) // Tip speed not applicable for elevator
//         ? (motorVelocityRPM / 60.0) * 2 * Math.PI * mechanismLengthMeters / gearRatio
//         : 0.0;
//     inputs.appliedVolts = appliedVolts;
//     inputs.amps = amps;
//     inputs.temperatureCelsius = 0.0; // Simulation does not model temperature

//     // Log debugging info
//     Logger.recordOutput("MotorIOSim/Type", sim.getClass().getSimpleName());
//     Logger.recordOutput("MotorIOSim/kP", pid.getP());
//   }

//   private double getFeedbackValue() {
//     if (sim instanceof SingleJointedArmSim armSim) {
//       return armSim.getAngleRads() / (2 * Math.PI);
//     } else if (sim instanceof DCMotorSim dcMotorSim) {
//       return dcMotorSim.getAngularPositionRad() / (2 * Math.PI);
//     } else if (sim instanceof ElevatorSim elevatorSim) { // Added ElevatorSim support
//       // Convert linear position to motor rotations
//       return (elevatorSim.getPositionMeters() / (2 * Math.PI * drumRadiusMeters)) * gearRatio;
//     }
//     return currentPositionRot;
//   }

//   @Override
//   public void setVoltage(double volts) {
//     isClosedLoop = false;
//     appliedVolts = volts;
//   }

//   @Override
//   public void setPercent(double percent) {
//     setVoltage(percent * 12.0); // Assume percent is normalized (-1 to 1)
//   }

//   @Override
//   public void setTargetMotorPosition(double motorRotations) {
//     isClosedLoop = true;
//     pid.setSetpoint(motorRotations);
//   }

//   @Override
//   public void setCurrentMotorPosition(double motorRotations) {
//     currentPositionRot = motorRotations;
//     if (sim instanceof SingleJointedArmSim armSim) {
//       armSim.setState(motorRotations * 2 * Math.PI, 0.0); // Reset velocity to 0
//     } else if (sim instanceof DCMotorSim dcMotorSim) {
//       dcMotorSim.setState(motorRotations * 2 * Math.PI, 0.0); // Reset velocity to 0
//     } else if (sim instanceof ElevatorSim elevatorSim) { // Added ElevatorSim support
//       // Convert motor rotations to linear position: rotations * (2 * π * drum_radius) / gear_ratio
//       double linearPositionMeters = (motorRotations * 2 * Math.PI * drumRadiusMeters) / gearRatio;
//       elevatorSim.setState(linearPositionMeters, 0.0); // Reset velocity to 0
//     }
//     // Other sim types: position is tracked internally
//   }

//   @Override
//   public void setTargetMechanismPosition(double mechanismRotations) {
//     setTargetMotorPosition(mechanismRotations * gearRatio);
//   }

//   @Override
//   public void setCurrentMechanismPosition(double mechanismRotations) {
//     setCurrentMotorPosition(mechanismRotations * gearRatio);
//   }

//   @Override
//   public void setTargetMotorVelocity(double motorRPM) {
//     isClosedLoop = true;
//     pid.setSetpoint(motorRPM / 60.0); // Convert RPM to rot/sec
//   }

//   @Override
//   public void setTargetMechanismVelocity(double mechanismRPM) {
//     setTargetMotorVelocity(mechanismRPM * gearRatio);
//   }

//   @Override
//   public void setTargetTipSpeed(double mps) {
//     if (mechanismLengthMeters <= 0 || sim instanceof ElevatorSim) { // Tip speed not applicable for elevator
//       Logger.recordOutput("MotorIOSim/Warning", "Tip speed control not supported: no mechanism length provided or elevator simulation");
//       return;
//     }
//     double mechanismRadPerSec = mps / mechanismLengthMeters;
//     setTargetMotorVelocity(Units.radiansPerSecondToRotationsPerMinute(mechanismRadPerSec) * gearRatio);
//   }

//   @Override
//   public void stop() {
//     isClosedLoop = false;
//     appliedVolts = 0.0;
//   }

//   @Override
//   public void setBrakeMode(boolean isBrake) {
//     // Simulation does not support brake/coast mode; no-op
//     Logger.recordOutput("MotorIOSim/Warning", "Brake mode not supported in simulation");
//   }

//   @Override
//   public void setTunerConstants(double p, double i, double d, double ff,
//                                 double iZone, double iMaxAccum,
//                                 double minOutput, double maxOutput, int slot) {
//     pid.setP(p);
//     pid.setI(i);
//     pid.setD(d);
//     // Feedforward, iZone, iMaxAccum, minOutput, maxOutput, and slot not supported
//     Logger.recordOutput("MotorIOSim/Warning", "Only P, I, D constants supported in simulation");
//   }

//   @Override
//   public void setMotionConstraints(double maxVel, double maxAccel, double allowedError, int slot) {
//     // Simulation does not support motion constraints; no-op
//     Logger.recordOutput("MotorIOSim/Warning", "Motion constraints not supported in simulation");
//   }

//   @Override
//   public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
//     // Simulation does not support current limits; no-op
//     Logger.recordOutput("MotorIOSim/Warning", "Current limits not supported in simulation");
//   }
// }
package frc.robot.lib.MotorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("rawtypes")
public class CompSim implements MotorIO {
  private static final double SIM_TIME_STEP_SECONDS = 0.02;
  private final LinearSystemSim sim;
  private final PIDController positionPid;
  private final PIDController velocityPid;
  private final double gearRatio;
  private final double mechanismLengthMeters;
  private boolean isClosedLoop = false;
  private boolean useVelocityControl = false;
  private double appliedVolts = 0.0;
  private double currentPositionRot = 0.0;

  /**
   * Constructor for MotorIOSim.
   *
   * @param sim The LinearSystemSim object (e.g., SingleJointedArmSim, FlywheelSim, DCMotorSim, ElevatorSim).
   * @param gearRatio The gear ratio from motor to mechanism (must be positive), including drum radius effects for elevators.
   * @param mechanismLengthMeters The length of the mechanism (e.g., arm length) for tip speed calculation, or 0 if not applicable.
   * @param positionPid PID controller for position control.
   * @param velocityPid PID controller for velocity control.
   */
  public CompSim(LinearSystemSim sim, double gearRatio, double mechanismLengthMeters,
                     PIDController positionPid, PIDController velocityPid) {
    if (gearRatio <= 0) {
      throw new IllegalArgumentException("Gear ratio must be positive");
    }
    if (mechanismLengthMeters < 0) {
      throw new IllegalArgumentException("Mechanism length cannot be negative");
    }
    this.sim = sim;
    this.gearRatio = gearRatio;
    this.mechanismLengthMeters = mechanismLengthMeters;
    this.positionPid = positionPid;
    this.velocityPid = velocityPid;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // Update simulation with applied voltage
    if (isClosedLoop) {
      double feedbackValue = useVelocityControl ? getVelocityFeedbackValue(inputs) : getPositionFeedbackValue();
      PIDController activePid = useVelocityControl ? velocityPid : positionPid;
      appliedVolts = activePid.calculate(feedbackValue);
    }
    sim.setInput(appliedVolts);
    sim.update(SIM_TIME_STEP_SECONDS);

    // Get position and velocity based on simulation type
    double motorVelocityRadPerSec = 0.0;
    double motorPositionRot = 0.0;
    double amps = 0.0;
    if (sim instanceof SingleJointedArmSim armSim) {
      motorPositionRot = armSim.getAngleRads() / (2 * Math.PI);
      motorVelocityRadPerSec = armSim.getVelocityRadPerSec();
      amps = armSim.getCurrentDrawAmps();
    } else if (sim instanceof FlywheelSim flywheelSim) {
      motorVelocityRadPerSec = flywheelSim.getOutput(0);
      motorPositionRot = currentPositionRot + (motorVelocityRadPerSec * SIM_TIME_STEP_SECONDS) / (2 * Math.PI);
      amps = flywheelSim.getCurrentDrawAmps();
    } else if (sim instanceof DCMotorSim dcMotorSim) {
      motorPositionRot = dcMotorSim.getAngularPositionRad() / (2 * Math.PI);
      motorVelocityRadPerSec = dcMotorSim.getAngularVelocityRadPerSec();
      amps = dcMotorSim.getCurrentDrawAmps();
    } else if (sim instanceof ElevatorSim elevatorSim) {
      double elevatorPositionMeters = elevatorSim.getPositionMeters();
      double elevatorVelocityMps = elevatorSim.getVelocityMetersPerSecond();
      // Convert linear position to motor rotations: meters * gearRatio
      motorPositionRot = elevatorPositionMeters * gearRatio;
      // Convert linear velocity to motor rad/s: m/s * gearRatio
      motorVelocityRadPerSec = elevatorVelocityMps * gearRatio;
      amps = elevatorSim.getCurrentDrawAmps();
    } else {
      Logger.recordOutput("MotorIOSim/Warning", "Unrecognized simulation type: " + sim.getClass().getSimpleName());
      motorVelocityRadPerSec = sim.getOutput(0);
      motorPositionRot = currentPositionRot + (motorVelocityRadPerSec * SIM_TIME_STEP_SECONDS) / (2 * Math.PI);
    }
    currentPositionRot = motorPositionRot;

    // Populate inputs
    inputs.motorPositionRot = currentPositionRot;
    inputs.motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(motorVelocityRadPerSec);
    inputs.mechanismPositionRot = currentPositionRot / gearRatio; // Rotations for non-elevator, meters for elevator
    inputs.mechanismVelocityRPM = inputs.motorVelocityRPM / gearRatio;
    inputs.tipSpeedMps = (mechanismLengthMeters > 0 && !(sim instanceof ElevatorSim))
        ? (motorVelocityRadPerSec / gearRatio) * mechanismLengthMeters
        : 0.0;
    inputs.appliedVolts = appliedVolts;
    inputs.amps = amps;
    inputs.temperatureCelsius = 0.0;

    // Log debugging info
    Logger.recordOutput("MotorIOSim/Type", sim.getClass().getSimpleName());
    Logger.recordOutput("MotorIOSim/PositionKp", positionPid.getP());
    Logger.recordOutput("MotorIOSim/VelocityKp", velocityPid.getP());
  }

  private double getPositionFeedbackValue() {
    if (sim instanceof SingleJointedArmSim armSim) {
      return armSim.getAngleRads() / (2 * Math.PI);
    } else if (sim instanceof DCMotorSim dcMotorSim) {
      return dcMotorSim.getAngularPositionRad() / (2 * Math.PI);
    } else if (sim instanceof ElevatorSim elevatorSim) {
      // Convert linear position to motor rotations
      return elevatorSim.getPositionMeters() * gearRatio;
    }
    return currentPositionRot;
  }

  private double getVelocityFeedbackValue(MotorIOInputs inputs) {
    if (sim instanceof SingleJointedArmSim armSim) {
      return armSim.getVelocityRadPerSec() / (2 * Math.PI);
    } else if (sim instanceof DCMotorSim dcMotorSim) {
      return dcMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    } else if (sim instanceof ElevatorSim elevatorSim) {
      // Convert linear velocity to motor rotations per second
      return elevatorSim.getVelocityMetersPerSecond() * gearRatio;
    } else if (sim instanceof FlywheelSim) {
      return sim.getOutput(0) / (2 * Math.PI);
    }
    return Units.rotationsPerMinuteToRadiansPerSecond(inputs.motorVelocityRPM);
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPercent(double percent) {
    setVoltage(percent * 12.0);
  }

  @Override
  public void setTargetMotorPosition(double motorRotations) {
    isClosedLoop = true;
    useVelocityControl = false;
    positionPid.setSetpoint(motorRotations);
  }

  @Override
  public void setCurrentMotorPosition(double motorRotations) {
    currentPositionRot = motorRotations;
    if (sim instanceof SingleJointedArmSim armSim) {
      armSim.setState(motorRotations * 2 * Math.PI, 0.0);
    } else if (sim instanceof DCMotorSim dcMotorSim) {
      dcMotorSim.setState(motorRotations * 2 * Math.PI, 0.0);
    } else if (sim instanceof ElevatorSim elevatorSim) {
      // Convert motor rotations to linear position: rotations / gearRatio
      double linearPositionMeters = motorRotations / gearRatio;
      elevatorSim.setState(linearPositionMeters, 0.0);
    }
  }

  @Override
  public void setTargetMechanismPosition(double mechanismPosition) {
    // mechanismPosition is rotations for non-elevator, meters for elevator
    setTargetMotorPosition(mechanismPosition * gearRatio);
  }

  @Override
  public void setCurrentMechanismPosition(double mechanismPosition) {
    setCurrentMotorPosition(mechanismPosition * gearRatio);
  }

  @Override
  public void setTargetMotorVelocity(double motorRPM) {
    isClosedLoop = true;
    useVelocityControl = true;
    velocityPid.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(motorRPM) / (2 * Math.PI));
  }

  @Override
  public void setTargetMechanismVelocity(double mechanismRPM) {
    setTargetMotorVelocity(mechanismRPM * gearRatio);
  }

  @Override
  public void setTargetTipSpeed(double mps) {
    if (mechanismLengthMeters <= 0 || sim instanceof ElevatorSim) {
      Logger.recordOutput("MotorIOSim/Warning", "Tip speed control not supported: no mechanism length provided or elevator simulation");
      return;
    }
    double mechanismRadPerSec = mps / mechanismLengthMeters;
    setTargetMotorVelocity(Units.radiansPerSecondToRotationsPerMinute(mechanismRadPerSec) * gearRatio);
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    Logger.recordOutput("MotorIOSim/Warning", "Brake mode not supported in simulation");
  }

  @Override
  public void setTunerConstants(double p, double i, double d, double ff,
                                double iZone, double iMaxAccum,
                                double minOutput, double maxOutput, int slot) {
    PIDController targetPid = (slot == 0) ? positionPid : velocityPid;
    targetPid.setP(p);
    targetPid.setI(i);
    targetPid.setD(d);
    Logger.recordOutput("MotorIOSim/Warning", "Only P, I, D constants supported in simulation");
  }

  @Override
  public void setMotionConstraints(double maxVel, double maxAccel, double allowedError, int slot) {
    Logger.recordOutput("MotorIOSim/Warning", "Motion constraints not supported in simulation");
  }

  @Override
  public void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
    Logger.recordOutput("MotorIOSim/Warning", "Current limits not supported in simulation");
  }
}
