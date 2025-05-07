package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.REV_Double;
import frc.robot.util.Rev_Hardware.REV_Follower;
import frc.robot.util.Rev_Hardware.REV_Max;


public class Neo_Double extends SubsystemBase { 



  public Neo_Double() {}

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor CAN ID
   * drive motor PID value P
   * rotation motor can ID
   * rotation motor PID value P
   * external CANCoder can ID
   * measured CANCoder offset
   */

   public final static REV_Max Leader = new REV_Max(
    REV_Double.MotorIdentification,
    REV_Double.Motor_int,
    REV_Double.Motor_double,
    REV_Double.Motor_boolean,
    REV_Double.TunnerValues,
    REV_Double.MotionConstants);

   public final static REV_Follower Follower = new REV_Follower(
    "Follower_bob", 
    11,
    Leader.get_EmergencyStopAmps(),
    Leader.get_Follower_Motor_doubles(),
    Leader.getConfig(),
    Leader.getMotor(),
    true,
    true);


  @Override
  public void periodic() {
    Logger.recordOutput(REV_Double.MotorIdentification+"/Motor_Position", Leader.getMotorPosition());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Motor_Velocity", Leader.getMotorVelocity());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Mech_Position", Leader.getMechanismPosition());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Mech_Velocity", Leader.getMechanismVelocity());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Amps", Leader.getAmps());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Volts", Leader.getVoltage());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Temp", Leader.getTemputerInCelcius());

    Logger.recordOutput("Follower_bob"+"/Motor_Position", Follower.getMotorPosition());
    Logger.recordOutput("Follower_bob"+"/Motor_Velocity", Follower.getMotorVelocity());
    Logger.recordOutput("Follower_bob"+"/Mech_Position", Follower.getMechanismPosition());
    Logger.recordOutput("Follower_bob"+"/Mech_Velocity", Follower.getMechanismVelocity());
    Logger.recordOutput("Follower_bob"+"/Amps", Follower.getAmps());
    Logger.recordOutput("Follower_bob"+"/Volts", Follower.getVoltage());
    Logger.recordOutput("Follower_bob"+"/Temp", Follower.getTemputerInCelcius());
  }

  }

