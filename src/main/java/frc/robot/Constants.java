package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;



/**
 * The Constants class is a place to store all the robot-wide numbers and settings.
 * These are values like motor speeds, sensor ports, or any fixed numbers the robot uses.
 *
 * <p>These values do not change while the robot is running—they stay the same
 * during a match or test.
 *
 * <p> When multiple robots are using the same code base,
 * this class helps keep shared settings in one place,
 * even if the robots behave slightly differently.
 *
 * <p>This class should only hold constants—no code that does anything (no functions or logic).
 * All constants should be declared as `static final` and written in `SNAKE_CASE`
 * (all uppercase letters with underscores between words).
 *
 * <p>It’s a good idea to use static imports for these constants when you need them,
 * so your code is cleaner and easier to read.
 */
public final class Constants {

  public static class BuildConstants {

    public static final String PROJECT_NAME = "MotorTrainerBoard";
    public static final String VERSION = "Teaching/Testing";
    public static final String GIT_REVISION = "1.2.0";
    public static final String GIT_BRANCH = "main";
    public static final String BUILD_DATE = "2025-08-23 11:01 AM EST";
    public static final int DIRTY = 0;
  }
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean tuningMode = false;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class RobotConstants {
        public static String serial;
        public static boolean isComp;
        public static boolean isAlpha;

        // TODO: Fill in with real serial number prefixes. Figure out by displaying/logging String
        // serial.
        public static final String compSerial = "0001";
        public static final String alphaSerial = "0000";

        static {
            if (Robot.isReal()) {
                // Roborio id recognition
                serial = System.getenv("serialnum");
            } else {
                serial = "5188";
            }
            RobotConstants.isComp = serial.startsWith(RobotConstants.compSerial);
            RobotConstants.isAlpha = serial.startsWith(RobotConstants.alphaSerial);
        }
    }

    public static RobotType robotType = RobotConstants.isComp ? RobotType.COMP
        : RobotConstants.isAlpha ? RobotType.ALPHA : RobotType.NONE;

    public enum RobotType {
        COMP,
        ALPHA,
        NONE
    }

 
  // public static class REV_Single {
  //   /** Unique identifier for the motor, used for logging*/
  //   public static final String MotorIdentification = "1_FlyWheel/";

  //   /** CAN ID assigned to the motor */
  //   public static final int Motor_ID = 14;

  //   /** Indicates if the motor is a Flex model */
  //   public static final boolean isFlex = false;

  //   /** Gear ratio from motor to end motion */
  //   public static final double Gear_Ratio = 100;

  //   /** Wheel radius in meters */
  //   public static final double Wheel_Radius = 0.0508;

  //   /** Idle mode setting (true for brake, false for coast) */
  //   public static final boolean Idle_Mode = false;

  //   /** Motor inversion setting */
  //   public static final boolean Inverted = false;

  //   /** Motion profile for real-world motor control */
  //   public static final REV_MotionProfile motionProfile_Real = new REV_MotionProfile(
  //     0.052554, 0.0, 0.029932, 0.000022105,
  //     0.0, 0.0, -0.75, 0.75,

  //     1.2939E-10, 0.00000015, 0.0, 0.0000815,
  //     75.0, 0.003, -0.75, 0.75,

  //     0, 0, 0, 0,
  //     0, 0, 0, 0,
      
  //     500, 750, 0.1
  //   );

  //   /** Motion profile for real-world motor control writen by Mang */
  //   public static final REV_MotionProfile motionProfile_mang = new REV_MotionProfile(
  //     0.93484, 0.0, 0.0091466, 0.00092549,
  //     0.0, 0.0, -0.75, 0.75,
  //     1.558E-06, 0, 0, 0.0000803,
  //     0, 0, -1, 1,
  //     0, 0, 0, 0,
  //     0, 0, 0, 0,
  //     500, 750, 1
  //   );

  //   /** Motion profile for simulation, defining parameters like cruise RPM and acceleration */
  //   public static final SIM_MotionProfile motionProfile_Sim = new SIM_MotionProfile(
  //     0, 0.0, 10,    
  //     0.0, 0.1076, 0.0,   

  //     0, 0.0, 0,    
  //     0.0, 0.1076, 0,   
      
  //     0, 

  //     100,  
  //     100
  //   );
  // }

  // public static class REV_Double {
  //   /** Unique identifier for the leader motor, used for logging*/
  //   public static final String MotorIdentification_Leader = "2_Climber/Leader";

  //   /** Unique identifier for the follower motor, used for logging*/
  //   public static final String MotorIdentification_Follower = "2_Climber/Follower";

  //   /** CAN ID assigned to the leader motor */
  //   public static final int Motor_ID_Leader = 10;

  //   /** CAN ID assigned to the follower motor */
  //   public static final int Motor_ID_Follower = 11;

  //   /** Indicates if the motors are Flex models */
  //   public static final boolean isFlex = false;

  //   /** Gear ratio from motor to end motion */
  //   public static final double Gear_Ratio = 300;

  //   /** Wheel radius in meters */
  //   public static final double Wheel_Radius = 0.0508;

  //   /** Idle mode setting (true for brake, false for coast) */
  //   public static final boolean Idle_Mode = true;

  //   /** Leader motor inversion setting */
  //   public static final boolean Leader_Inverted = false;

  //   /** Follower motor inversion relative to the leader */
  //   public static final boolean Follower_Inverted_from_Leader = true;

  //   /** Indicates if the follower motor is a Spark controller */
  //   public static final boolean Follower_Spark = true;

  //   /** Motion profile for real-world motor control */
  //   public static final REV_MotionProfile motionProfile_Real = new REV_MotionProfile(
  //     1, 0.0, 0, 0,
  //     0.0, 0.0, -1, 1,

  //     1.2939E-10, 0.00000015, 0.0, 0.0000815,
  //     75.0, 0.003, -1, 1,

  //     0, 0, 0, 0,
  //     0, 0, 0, 0,

  //     1500, 750, 0.1
  //   );

  //   /** Motion profile for simulation, defining parameters like cruise RPM and acceleration */
  //   public static final SIM_MotionProfile motionProfile_Sim = new SIM_MotionProfile(
  //     0, 0, 0,    
  //     0, 0.3227, 0,  

  //     0, 0.0, 0,    
  //     0, 0.3227, 0,   
      
  //     0, 

  //     30, 60
  //   );
  // }

  // public static class Kraken_Motor {

  //   public static final String MotorIdentification_LEADER = "BOB";// Change this to a helpful identification name
  //   public static final String MotorIdentification_FOLLOWER = "BOB2";// Change this to a helpful identification name
  //   public static final String CANLoop = "rio";// Idenify which CAN loop motor is on

  //   public static final int Motor_ID_LEADER = 1;
  //   public static final int Motor_ID_FOLLOWER = 2;

  //   public static final double Gear_Ratio = 1; // Gear Ratio from motor to end motion
  //   public static final double Wheel_Radius = 0; // Wheel Radius in Meter

  //   public static final boolean Idle_Mode = true;
  //   public static final boolean LEADER_Inverted = false;
  //   public static final boolean Inverted_FROM_LEADER = false;
    

  //   public static final double[] MotionProfile = MotorConstants.CTRE_createMotionProfile(
  //       0.0, // 0 Position PID kP
  //       0.0, // 1 Position PID kI
  //       0.0, // 2 Position PID kD
  //       0.0, // 3 Position FF kS
  //       0.0, // 4 Position FF kV
  //       0.0, // 5 Position FF kA
  //       0.0, // 6 Reverse FF 0 = false or 1= true
  //       0.0, // 7 Position FF Type

  //       // Options
  //       // 0 = normal forces alwasy opposses motion
  //       // 1=constant in one derection motion elavator
  //       // 2 = changing like a pivoit or arm

  //       0.0, // 8 Position PID kP
  //       0.0, // 9 Position PID kI
  //       0.0, // 10 Position PID kD
  //       0.0, // 11 Position FF kS
  //       0.0, // 12 Position FF kV
  //       0.0, // 13 Position FF kA

  //       0.0, // 14 Cruise Velocity
  //       0.0, // 15 Acceleration
  //       0.0, // 16 Jerk
  //       0.0, // 17 Motion kV
  //       0.0 // 18 Motion kA
  //   );

  //   public static final CTRE_MotionProfile MotionProfileV2 = new CTRE_MotionProfile(
  //     // Position PID gains
  //     0, 0, 0,
  //     // Feedforward kS, kV, kA
  //     0, 0, 0,
  //     // Gravity type code (0 = none)
  //     0,
  //     // Velocity PID gains
  //     0, 0, 0,
  //     // Velocity feedforward kS, kV, kA
  //     0, 0, 0,
  //     // Motion constraints: cruiseVelocity, acceleration, jerk
  //     0, 0, 0,
  //     // Motion feedforward constants
  //     0, 0
  // );

//  }

//   public class ArmConstants {
//     public static String NAME = "Rotary";

//     public static boolean isFlex = false;

//     public static int LEADER_ID = 1;
//     public static int FOLLOWER_ID = 2;

//     public static final Angle TOLERANCE = Degrees.of(2.0);

//     public static final AngularVelocity CRUISE_VELOCITY = Units.RadiansPerSecond.of(1);
//     public static final AngularAcceleration ACCELERATION = CRUISE_VELOCITY.div(0.1).per(Units.Second);
//     public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

//     private static final double ROTOR_TO_SENSOR = (2.0 / 1.0);
//     private static final double SENSOR_TO_MECHANISM = (2.0 / 1.0);

//     public static final Translation3d OFFSET = Translation3d.kZero;

//     public static final Angle MIN_ANGLE = Degrees.of(0.0);
//     public static final Angle MAX_ANGLE = Rotations.of(.5);
//     public static final Angle STARTING_ANGLE = Rotations.of(0.0);
//     public static final Distance ARM_LENGTH = Meters.of(1.0);

//     public static final RotaryMechCharacteristics CONSTANTS =
//         new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

//     public static final Mass ARM_MASS = Kilograms.of(.01);
//     public static final DCMotor DCMOTOR = DCMotor.getNeo550(1);
//     public static final MomentOfInertia MOI = KilogramSquareMeters
//         .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));


//        public static SparkMaxConfig configRev(){

//         //Change this to mathc the encoder you are working with
//         SparkMaxConfig config = new SparkMaxConfig();

//         config.smartCurrentLimit(
//             20,
//             35,
//             60
//         )
//         .inverted(false)
//         .idleMode(true ? IdleMode.kBrake : IdleMode.kCoast)
//         .signals.primaryEncoderPositionPeriodMs(20);
        

//         config.closedLoop
//             // Position slot 0
//             .p(5, ClosedLoopSlot.kSlot0)
//             .i(0, ClosedLoopSlot.kSlot0)
//             .d(0.25, ClosedLoopSlot.kSlot0)
//             .iZone(0, ClosedLoopSlot.kSlot0)
//             .iMaxAccum(0, ClosedLoopSlot.kSlot0)
//             .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
//             .velocityFF(0.0,ClosedLoopSlot.kSlot0)

//             .p(10, ClosedLoopSlot.kSlot1)
//             .i(0, ClosedLoopSlot.kSlot1)
//             .d(0, ClosedLoopSlot.kSlot1)
//             .velocityFF(0, ClosedLoopSlot.kSlot1)
//             .iZone(0, ClosedLoopSlot.kSlot1)
//             .iMaxAccum(0, ClosedLoopSlot.kSlot1)
//             .outputRange(-1,1, ClosedLoopSlot.kSlot1)

            

//         .maxMotion
//             .maxAcceleration(1,ClosedLoopSlot.kSlot0)
//             .maxVelocity(1,ClosedLoopSlot.kSlot0);

//         config.softLimit
//         .forwardSoftLimit(0.05)
//         .forwardSoftLimitEnabled(false)
//         .reverseSoftLimit(0)
//         .reverseSoftLimitEnabled(false);

//         //Place gear ratio in this
//         config.encoder
//         .positionConversionFactor(1)
//         .velocityConversionFactor(1); 


//             return config;
//     }

//     public static RotaryMechanismReal getReal()
//     {
//         return new RotaryMechanismReal(
//             new RevMotorIO(NAME,
//             LEADER_ID,
//             isFlex,
//             configRev()//,
//             //new RevFollowerFollower(FOLLOWER_ID, true)
            
//               ),
//             CONSTANTS
//             );
//     }

//     public static RotaryMechanismSim getSim()
//     {
//       return new RotaryMechanismSim(
//         new MotorIOSparkSim(
//         NAME,
//         LEADER_ID,
//         isFlex,
//         configRev(),
//         DCMOTOR//,
//         //new RevFollowerFollower(FOLLOWER_ID, true)
//           ),
//           DCMOTOR, MOI, false, CONSTANTS
//         );
//     }

//     public static RotaryMechanism getReplay()
//     {
//         return new RotaryMechanism(NAME, CONSTANTS) {};
//     }
// }
 }