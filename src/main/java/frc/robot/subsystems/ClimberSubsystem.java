package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Motor.MotorTypes;
import frc.robot.lib.MotorV3.*;
import frc.robot.Constants.REV_Double;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

/** Subsystem class for controlling the climber mechanism */
public class ClimberSubsystem extends SubsystemBase {

  /** The leader motor controller for the climber */
  public final MotorIO leaderMotor;

  /** Inputs for the leader motor, including amps, voltage, position, velocity, etc. */
  private final MotorIOInputsAutoLogged leaderInputs = new MotorIOInputsAutoLogged();

  /** Constructor run when the class is instantiated (executes once at code start) */
  public ClimberSubsystem() {
    // Check if the robot is running in real or simulation mode
    if (RobotBase.isReal()) {
      // Initialize leader motor for real hardware with Neo motor on REV controller
      leaderMotor = new RevMotorIO(
        REV_Double.Motor_ID_Leader,
        REV_Double.isFlex,
        REV_Double.Gear_Ratio,
        REV_Double.Wheel_Radius,
        REV_Double.Idle_Mode,
        REV_Double.Leader_Inverted,
        new MotorTypes.Neo(),
        REV_Double.motionProfile_Real
      );

      // Initialize follower motor, slaved to the leader motor
      leaderMotor.set_Follower(REV_Double.Motor_ID_Follower, false, REV_Double.Follower_Inverted_from_Leader);

    } else {
      // Initialize leader motor for simulation with Neo550 motors
      leaderMotor = new MotorIOSim(
        DCMotor.getNeo550(2),
        0.4,
        REV_Double.Gear_Ratio,
        REV_Double.Wheel_Radius,
        REV_Double.motionProfile_Sim,
        false
      );
    }
  }

  /** Runs on every code loop to update motor states and log data */
  @Override
  public void periodic() {
    // Update inputs for both leader and follower motors
    leaderMotor.updateInputs(leaderInputs);

    // Log motor inputs for debugging and analysis
    Logger.processInputs("2_NeoLeader", leaderInputs);

    //Record the current position of the climber
    Logger.recordOutput("Pose_Climber", new Pose3d[] {new Pose3d(-0.127,0,0.1143, new Rotation3d(0, Units.rotationsToRadians(leaderInputs.mechanismPositionRot), 0))});

    //if(leaderInputs.mechanismPositionRot<-0.3){Leader.setVoltage(0.6);}
    //else if(leaderInputs.mechanismPositionRot>-0.01){Leader.setVoltage(-0.6);} 0-0.45

    leaderMotor.setVoltage(12);
  }

}
