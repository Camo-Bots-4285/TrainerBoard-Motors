package frc.robot;

import frc.robot.configs.MotorConstants;
import frc.robot.configs.RobotConstants;



 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
 public final class Constants {

    /*This is used to change the constants for each robot */
    public static final RobotIdentity identity = RobotIdentity.Robot_3;
    public static RobotConstants config = RobotConstants.getRobotConstants(identity);

    public enum RobotIdentity {
      Robot_1,
      Robot_2,
      Robot_3
    }

    public static class BuildConstants {

      public static final String PROJECT_NAME = "MotorTrainer_Board";
      public static final String VERSION = "unspecified";
      public static final String GIT_REVISION = "2.0";
      public static final String GIT_BRANCH = "main";
      public static final String BUILD_DATE = "2025-06-10 7:26 PM EST";
      public static final int DIRTY = 0;
    }

 
    //Here is an example of how to set up constats for a Neo 550
  public static class REV_Single{

    public static final boolean ENABLE_SINGLE_NEO = true;

    /*Motor location */
    public static final String MotorIdentification = "Single Motor";//Change this to a helpful identify the motor when priting values

    public static final int Motor_ID = 14;

    public static final double Gear_Ratio = 0.01; // Gear Ratio from motor to end motion
    public static final double Wheel_Radius = 0.0508; // Wheel Radius in Meter

    public static final boolean Idle_Mode = false;
    public static final boolean Inverted = false;


    public static final double[] MotionProfile = MotorConstants.REV_createMotionProfile(
      0.052554, //0  Position PID kP 1.5029 0.052554
      0.0, //1  Position PID kI
      0.029932, //2  Position PID kD 134.14 0.029932
      0.000022105, //3  Position FF  kV
      0.0, //4  Position iZone
      0.0, //5  Position iMaxAccumulation
      -0.75, //6  Position OutPut Min -1 _ 0
      0.75, //7  Position OutPut Max 0 _ 1


      1.2939E-10, //8  Velocity PID kP //0.00035 1.2939E-10
      0.00000015, //9  Velocity PID kI
      0.0, //10  Velocity PID kD 0.005
      0.0000815, //11  Velocity FF  kV  //0.0000815 0.000022105
      75.0, //12  Velocity iZone
      0.003, //13  Velocity iMaxAccumulation
      -0.75, //14  Velocity OutPut Min
      0.75, //15  Velocity OutPut Max

      0, //16  Velocity PID kP
      0, //17  Velocity PID kI
      0, //18  Velocity PID kD
      0, //19  Velocity FF  kV
      0, //20  Velocity iZone
      0, //21  Velocity iMaxAccumulation
      0, //22  Velocity OutPut Min
      0, //23  Velocity OutPut Max

      1500, //24 max Velocity RPM
      750, //25 max Acceleration RPM^2
      0.1 //26 allowed Closed Loop Error Rotations 
    );

    // 0.00035, //8  Velocity PID kP //0.00035
    //   0.0, //9  Velocity PID kI
    //   0.005, //10  Velocity PID kD 0.005
    //   0.0000815, //11  Velocity FF  kV  //0.0000815
    //   0.0, //12  Velocity iZone
    //   0.0, //13  Velocity iMaxAccumulation
    //   -0.75, //14  Velocity OutPut Min
    //   0.75, //15  Velocity OutPut Max

  }


  public static class REV_Double{

    public static final boolean ENABLE_DOUBLE_NEO = true;

    /*Motor location */
    public static final String MotorIdentification_Leader = "Double_Neo_Follower";//Change this to a helpful identify the motor when priting values
    public static final String MotorIdentification_Follower = "Double_Neo_Follower";//Change this to a helpful identify the motor when priting values

    public static final int Motor_ID_Leader = 10;
    public static final int Motor_ID_Follower = 11;

    public static final double Gear_Ratio = (1/300); // Gear Ratio from motor to end motion
    public static final double Wheel_Radius = 0.0508; // Wheel Radius in Meter

    public static final boolean Idle_Mode = true;
    public static final boolean Leader_Inverted = false;
    public static final boolean Follower_Inverted_from_Leader = true;
    public static final boolean Follower_Spark = true;

    public static final double[]  MotionProfile = MotorConstants.REV_createMotionProfile(
      1, //0  Position PID kP
      0, //1  Position PID kI
      0, //2  Position PID kD
      0, //3  Position FF  kV
      0, //4  Position iZone
      0, //5  Position iMaxAccumulation
      -0.1, //6  Position OutPut Min
      0.1, //7  Position OutPut Max


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
 
      2, //24 max Velocity
      3, //25 max Acceleration
      0.1 //26 allowed Closed Loop Error
    );

  }

 
    //The following code it an example for a Falcon
  public static class Falcon_Motor{

  public static final String MotorIdentification = "BOB";//Change this to a helpful identification name
  public static final String CANLoop = "rio";// Idenify which CAN loop motor is on

    public static final int Motor_ID = 1;

    public static final double Gear_Ratio = 1; // Gear Ratio from motor to end motion
    public static final double Wheel_Radius = 0; // Wheel Radius in Meter

    public static final boolean Idle_Mode = true;
    public static final boolean Inverted = false;


  public static final double[] MotionProfile = MotorConstants.CTRE_createMotionProfile(
    0.0, //0  Position PID kP
    0.0, //1  Position PID kI
    0.0, //2  Position PID kD
    0.0, //3  Position FF  kS
    0.0, //4  Position FF  kV
    0.0, //5  Position FF  kA
    0.0, //6  Reverse FF 0 = false or 1= true
    0.0, //7 Position FF Type

    //Options 
    //0 = normal forces alwasy opposses motion
    //1=constant in one derection motion elavator
    //2 = changing like a pivoit or arm

    0.0, //8  Position PID kP
    0.0, //9  Position PID kI
    0.0, //10  Position PID kD
    0.0, //11 Position FF  kS
    0.0, //12  Position FF  kV
    0.0, //13  Position FF  kA

    0.0, //14 Cruise Velocity
    0.0, //15 Acceleration
    0.0, //16 Jerk
    0.0, //17 Motion kV
    0.0 //18 Motion kA
  );

  }


 }