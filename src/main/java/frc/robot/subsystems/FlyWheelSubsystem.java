package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Motor.MotorTypes;
import frc.robot.lib.MotorV3.*;
import static frc.robot.Constants.REV_Single;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import org.littletonrobotics.junction.Logger;

public class FlyWheelSubsystem extends SubsystemBase {

  /**The motor class that can be used to control the motor in real or simulation */
  public final MotorIO FlyWheel_Motor;

  /**inputs to the motor class you can find thins like amps, voltage, postion, velocity, etc */
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  /** Constructor run when class is defined (run only once when the code starts) */
  public FlyWheelSubsystem() {
    //Checks if the robot is real
    FlyWheel_Motor = RobotBase.isReal()
    //If it is real runs code for a Neo550 running on a Spark
        ?  new RevMotorIO(
          REV_Single.Motor_ID,
          REV_Single.isFlex,
          REV_Single.Gear_Ratio,
          REV_Single.Wheel_Radius,
          REV_Single.Idle_Mode,
          REV_Single.Inverted,
          new MotorTypes.Neo550(),
          REV_Single.motionProfile_Real)
    //If is not real will run a FlyWheel that is driven by a Neo 550
        : new FlyWheelSimIO(
          DCMotor.getNeo550(1),
          REV_Single.Gear_Ratio,
          REV_Single.Wheel_Radius,
          0.4,
          REV_Single.motionProfile_Sim);

      //Set the motor encoder to read zero on start of code
      FlyWheel_Motor.setCurrentMechanismPosition(0);
  }

  //Will run on every code loop
  @Override
  public void periodic() {

    //Updated the motor inputs 
    FlyWheel_Motor.updateInputs(inputs);
    //Logs all input to the string under varable name REV_Single.MotorIdentification
    Logger.processInputs(REV_Single.MotorIdentification, inputs);

    //This code is used to help visulize the robot in simulation,logs, and replay
    //https://docs.advantagescope.org/more-features/custom-assets

    //Logs a zero position for mechanism calibration
    Logger.recordOutput("Pose_ZeroedCompont", new Pose3d[] {new Pose3d()});
    //Record the current position of the wheel
    Logger.recordOutput("Pose_Wheel", new Pose3d[] {new Pose3d(0.04445,-0.127,0.6223, new Rotation3d(0, 0, - Units.rotationsToRadians(inputs.mechanismPositionRot)))});

    FlyWheel_Motor.setVoltage(12);
  }

}

