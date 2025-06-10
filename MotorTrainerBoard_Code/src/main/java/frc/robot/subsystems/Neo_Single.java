package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.REV_Single;
import frc.robot.configs.MotorConstants;
import frc.robot.lib.Rev_Hardware.REV_Max;


public class Neo_Single extends SubsystemBase { 

  public Neo_Single() {}

   public final static REV_Max Single_Motor = new REV_Max(
    REV_Single.MotorIdentification,
    MotorConstants.Neo550_int(REV_Single.Motor_ID),
    MotorConstants.Neo500_double(REV_Single.Gear_Ratio, REV_Single.Wheel_Radius),
    REV_Single.Idle_Mode,
    REV_Single.Inverted,
    REV_Single.MotionProfile);

  @Override
  public void periodic() {
    Logger.recordOutput(REV_Single.MotorIdentification+"/Motor_Position", Single_Motor.getMotorPosition().getRotations());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Motor_Velocity", Single_Motor.getMotorVelocity_RPM().getRotations());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Mech_Position", Single_Motor.getMechanismPosition().getRotations());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Mech_Velocity", Single_Motor.getMechanismVelocity_RPM().getRotations());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Amps", Single_Motor.getAmps());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Volts", Single_Motor.getVoltage());
    Logger.recordOutput(REV_Single.MotorIdentification+"/Temp", Single_Motor.getTemputerInCelcius());

  }

}

