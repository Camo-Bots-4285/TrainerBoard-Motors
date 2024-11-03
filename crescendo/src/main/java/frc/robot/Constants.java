// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;


 

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
  public static boolean isRed= true;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
    /*
    Makes robot more consistent by making it act like it only has 12 volts thw whole time 
    This is new for 2025    Should be eqaul to the battery voltage before climb
    */
    public static final int VOLTAGE_COMP_ROBOT = 12;
  }


  public static class SingleMotorConstants {

    public static final int SINGLE_MOTOR = 13;
    public static final int MAX_AMPS = 35;
    public static final int VOLTAGE_COMP_SINGLE_MOTOR = OperatorConstants.VOLTAGE_COMP_ROBOT;

    public static PIDController PID_Positon_Single_Motor = new PIDController(0.285, 0, 0.0);

    public static final double WheelRadius = 5;
    public static final double GEAR_RATIO = 1.5;

  }

   public static class TwoMotorConstants {

    public static final int LEADER_MOTOR = 13;
    public static final int FOLLOWER_MOTOR = 14;
    public static final int MAX_AMPS = 35;
    public static final int VOLTAGE_COMP_TWO_MOTOR = OperatorConstants.VOLTAGE_COMP_ROBOT;
    
    public static PIDController PID_Positon_TWO_Motor = new PIDController(0.285, 0, 0.0);

    public static final double WheelRadius = 5;
    public static final double GEAR_RATIO = 1.5;

  }

  
  public static class IntakeConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_INTAKE_FLOOR = 13;
    public static final int MOTOR_INTAKE_FLOOR2 = 14;

  }
 }

