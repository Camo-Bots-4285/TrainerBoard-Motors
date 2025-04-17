package frc.robot;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotContainer;



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

 

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

  public static class REV_Motor_Charactoristics{

    public static final int[] Neo_int={
      0, //PLeace hold for motor so number line up
      40, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      7, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      60  //4 Emergency current limit
    };
    public static final double[] Neo_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    public static final int[] Neo550_int={
      0,//PLeace hold for motor so number line up
      35, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      5, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      50  //4 Emergency current limit
    };
    public static final double[] Neo550_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    public static final int[] NeoVortex_int={
      0,//PLeace hold for motor so number line up
      40, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      5, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      55  //4 Emergency current limit
    };
    public static final double[] NeoVortex_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    
    }
 
    //Here is an example of how to set up constats for a Neo 550
  public static class REV_Single{

    /*Motor location */
    public static final String MotorIdentification = "PlaceHolderName";//Change this to a helpful identify the motor when priting values

    public static final int[] Motor_int ={
    14, //0 Motor ID
    REV_Motor_Charactoristics.Neo_int[1], //1 Stall current limit
    REV_Motor_Charactoristics.Neo_int[2], //2 FreeSpin 
    REV_Motor_Charactoristics.Neo_int[3], //3 LimitRPM
    REV_Motor_Charactoristics.Neo_int[4]  //4 Emergency current limit
    };

    /* Motion Tracking Constants*/
    public static final double[] Motor_double = {//Start counting at zero and every number after that is positive plus one
      0.01, //0  Gear Ratio
      0.0508, //1  Wheel Radius [meters]
      REV_Motor_Charactoristics.Neo_double[2]//2 Tempurature OverHeat Warning will call
    }; 

    public static final boolean[] Motor_boolean = {//Start counting at zero and every number after that is positive plus one
      //Break mode is true 
      true, //0  Active Motor State
      false, //1 Invert Motor

    }; 

    public static final double[] TunnerValues = {
      1.0, //0  Position PID kP
      0.0, //1  Position PID kI
      0.0, //2  Position PID kD
      0.0, //3  Position FF  kV
      0.0, //4  Position iZone
      0.00, //5  Position iMaxAccumulation
      -0.75, //6  Position OutPut Min -1 _ 0
      0.75, //7  Position OutPut Max 0 _ 1


      0.00035, //8  Velocity PID kP //0.00035
      0.0, //9  Velocity PID kI
      0.005, //10  Velocity PID kD 0.005
      0.0000815, //11  Velocity FF  kV  //0.0000815
      0.0, //12  Velocity iZone
      0.0, //13  Velocity iMaxAccumulation
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
 
    };

    public static final double[] MotionConstants = {
      1250, //0 max Velocity RPM
      750, //1 max Acceleration RPM^2
      0.5, //2 allowed Closed Loop Error Rotations
    };

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

    /*Motor location */
    public static final String MotorIdentification = "PlaceHolderName";//Change this to a helpful identify the motor when priting values

    public static final int[] Motor_int ={
    10, //0 Motor ID
    REV_Motor_Charactoristics.Neo_int[1], //1 Stall current limit
    REV_Motor_Charactoristics.Neo_int[2], //2 FreeSpin 
    REV_Motor_Charactoristics.Neo_int[3], //3 LimitRPM
    REV_Motor_Charactoristics.Neo_int[4]  //4 Emergency current limit
    };

    /* Motion Tracking Constants*/
    public static final double[] Motor_double = {//Start counting at zero and every number after that is positive plus one
      (1/300), //0  Gear Ratio
      2, //1  Wheel Radius [meters]
      REV_Motor_Charactoristics.Neo_double[2]//2 Tempurature OverHeat Warning will call
    }; 

    public static final boolean[] Motor_boolean = {//Start counting at zero and every number after that is positive plus one
      //Break mode is true 
      true, //0  Active Motor State
      false, //1 Invert Motor

    }; 

    public static final double[] TunnerValues = {
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
 
    };

    public static final double[] MotionConstants = {
      2, //0 max Velocity
      3, //1 max Acceleration
      0.1, //2 allowed Closed Loop Error
    };

  }

  public static class CTRE_Motor_Charactoristics{

    public static final int[] Kraken60_int ={
      0, //0 Pleace holder for motor ID
      50, //1 Stator Current Limit max amps for the motor
      55, //2 SupplyCurrentLimit pulls this till lower currnt is applies
      40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
      70  //4 Emergency Stop Current
    
      };
      public static final double[] Kraken60_double = {//Start counting at zero and every number after that is positive plus one
        0, //0  Place holder for Gear Ration
        0, //1  Pleace holder for Wheelradius
        75,//2 Tempurature OverHeat Warning will call
        0.75//3 Supply Curren Time amount of time motor pulls max amps
      }; 

      public static final int[] Kraken44_int ={
        0, //0 Pleace holder for motor ID
        50, //1 Stator Current Limit max amps for the motor
        45, //2 SupplyCurrentLimit pulls this till lower currnt is applies
        40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
        60  //4 Emergency Stop Current
      
        };
        public static final double[] Kraken44_double = {//Start counting at zero and every number after that is positive plus one
          0, //0  Place holder for Gear Ration
          0, //1  Pleace holder for Wheelradius
          0,//2 Tempurature OverHeat Warning will call
          0//3 Supply Curren Time amount of time motor pulls max amps
        }; 

        public static final int[] Falcon_int ={
          0, //0 Pleace holder for motor ID
          50, //1 Stator Current Limit max amps for the motor
          45, //2 SupplyCurrentLimit pulls this till lower currnt is applies
          40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
          0  //4 Emergency Stop Current
        
          };
          public static final double[] Falcon_double = {//Start counting at zero and every number after that is positive plus one
            0, //0  Place holder for Gear Ration
            0, //1  Pleace holder for Wheelradius
            0,//2 Tempurature OverHeat Warning will call
            0//3 Supply Curren Time amount of time motor pulls max amps
          };  
    
 
    }

    //The following code it an example for a kraken 60
  public static class Kraken_Motor{

 
  public static final String CANLoop = "rio";// Idenify which CAN loop motor is on
  public static final String MotorIdentification = "PlaceHolderName";//Change this to a helpful identification name


  public static final int[] Motor_int ={
  1, //0 Motor ID
  CTRE_Motor_Charactoristics.Kraken60_int[1], //1 Stator Current Limit max amps for the motor
  CTRE_Motor_Charactoristics.Kraken60_int[2], //2 SupplyCurrentLimit pulls this till lower currnt is applies
  CTRE_Motor_Charactoristics.Kraken60_int[3],  //3 SupplyCurrentLimtLower continous current should not be higher then breaker
  CTRE_Motor_Charactoristics.Kraken60_int[4]  //4 Emergency Stop Current

  };

  /* Motion Tracking Constants*/
  public static final double[] Motor_double = {//Start counting at zero and every number after that is positive plus one
    0, //0  Gear Ratio
    0, //1  Wheel Radius [meters]
    CTRE_Motor_Charactoristics.Kraken60_double[2],//2 Tempurature OverHeat Warning will call
    CTRE_Motor_Charactoristics.Kraken60_double[3]//3 Supply Curren Time amount of time motor pulls max amps
  }; 

  public static final boolean[] Motor_boolean = {//Start counting at zero and every number after that is positive plus one
    //Break mode is true 
    true, //0  Active Motor State
    false, //1 Invert Motor

  }; 

  public static final double[] TunnerValues = {
    0, //0  Position PID kP
    0, //1  Position PID kI
    0, //2  Position PID kD
    0, //3  Position FF  kS
    0, //4  Position FF  kV
    0, //5  Position FF  kA
    0, //6  Reverse FF 0 = false or 1= true
    0, //7 Position FF Type
    //Options 
    //0 = normal forces alwasy opposses motion
    //1=constant in one derection motion elavator
    //2 = changing like a pivoit or arm

    0, //8  Position PID kP
    0, //9  Position PID kI
    0, //10  Position PID kD
    0, //11 Position FF  kS
    0, //12  Position FF  kV
    0, //13  Position FF  kA
  };

  public static final double[] MotionConstants = {
    0, //0 Cruise Velocity
    0, //1 Acceleration
    0, //2 Jerk
    0, //3 Motion kV
    0, //4 Motion kA
  };

  }




    }