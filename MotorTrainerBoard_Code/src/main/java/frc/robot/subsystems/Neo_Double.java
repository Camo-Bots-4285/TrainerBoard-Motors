package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.REV_Double;
import frc.robot.configs.MotorConstants;
import frc.robot.lib.Rev_Hardware.REV_Follower;
import frc.robot.lib.Rev_Hardware.REV_Max;


public class Neo_Double extends SubsystemBase { 



  public Neo_Double() {}


   public final static REV_Max Leader = new REV_Max(
    REV_Double.MotorIdentification_Leader,
    MotorConstants.Neo550_int(REV_Double.Motor_ID_Leader),
    MotorConstants.Neo500_double(REV_Double.Gear_Ratio, REV_Double.Wheel_Radius),
    REV_Double.Idle_Mode,
    REV_Double.Leader_Inverted,
    REV_Double.MotionProfile);

   public final static REV_Follower Follower = new REV_Follower(
    REV_Double.MotorIdentification_Follower, 
    REV_Double.Motor_ID_Follower,
    Leader.get_EmergencyStopAmps(),
    Leader.get_Follower_Motor_doubles(),
    Leader.getConfig(),
    Leader.getMotor(),
    REV_Double.Follower_Inverted_from_Leader,
    REV_Double.Follower_Spark);


  @Override
  public void periodic() {
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Motor_Position", Leader.getMotorPosition().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Motor_Velocity", Leader.getMotorVelocity_RPM().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Mech_Position", Leader.getMechanismPosition().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Mech_Velocity", Leader.getMechanismVelocity_RPM().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Amps", Leader.getAmps());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Volts", Leader.getVoltage());
    Logger.recordOutput(REV_Double.MotorIdentification_Leader+"/Temp", Leader.getTemputerInCelcius());

    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Motor_Position", Follower.getMotorPosition().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Motor_Velocity", Follower.getMotorVelocity_RPM().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Mech_Position", Follower.getMechanismPosition().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Mech_Velocity", Follower.getMechanismVelocity_RPM().getDegrees());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Amps", Follower.getAmps());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Volts", Follower.getVoltage());
    Logger.recordOutput(REV_Double.MotorIdentification_Follower+"/Temp", Follower.getTemputerInCelcius());
  }

  }

