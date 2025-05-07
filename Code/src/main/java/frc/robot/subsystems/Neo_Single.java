package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.REV_Double;
import frc.robot.Constants.REV_Single;
import frc.robot.util.Rev_Hardware.REV_Follower;
import frc.robot.util.Rev_Hardware.REV_Max;


public class Neo_Single extends SubsystemBase { 



  public Neo_Single() {}

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
    REV_Single.MotorIdentification,
    REV_Single.Motor_int,
    REV_Single.Motor_double,
    REV_Single.Motor_boolean,
    REV_Single.TunnerValues,
    REV_Single.MotionConstants);



  @Override
  public void periodic() {
    Logger.recordOutput(REV_Double.MotorIdentification+"/Motor_Position", Leader.getMotorPosition());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Motor_Velocity", Leader.getMotorVelocity());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Mech_Position", Leader.getMechanismPosition());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Mech_Velocity", Leader.getMechanismVelocity());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Amps", Leader.getAmps());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Volts", Leader.getVoltage());
    Logger.recordOutput(REV_Double.MotorIdentification+"/Temp", Leader.getTemputerInCelcius());
  }

  }

