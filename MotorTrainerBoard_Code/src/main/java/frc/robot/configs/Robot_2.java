package frc.robot.configs;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class Robot_2 implements RobotConstants {


  public Robot_2() {}

  @Override
  public String ConstantsName(){
    return "Robot_2";
  }

  @Override
  public double[] Single_Neo_MotionProfile() {

    double[] MotionProfile =
    
    {
      2, //0  Position PID kP
      0, //1  Position PID kI
      0, //2  Position PID kD
      0, //3  Position FF  kV
      0, //4  Position iZone
      0, //5  Position iMaxAccumulation
      0, //6  Position OutPut Min
      0, //7  Position OutPut Max

      0, //8  Velocity PID kP
      0, //9  Velocity PID kI
      0, //10  Velocity PID kD
      0, //11  Velocity FF  kV
      0, //12  Velocity iZone
      0, //13  Velocity iMaxAccumulation
      0, //14  Velocity OutPut Min
      0, //15  Velocity OutPut Max

      0, //16  Velocity PID kP
      0, //17  Velocity PID kI
      0, //18  Velocity PID kD
      0, //19  Velocity FF  kV
      0, //20  Velocity iZone
      0, //21  Velocity iMaxAccumulation
      0, //22  Velocity OutPut Min
      0, //23  Velocity OutPut Max
 
      0, //24 max Velocity
      0, //25 max Acceleration
      0, //26 allowed Closed Loop Error
    };
      return MotionProfile;
      
  }


}