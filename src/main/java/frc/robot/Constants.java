package frc.robot;

import frc.lib.util.Device;
import frc.lib.util.PID;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIORev;
import frc.lib.io.motor.MotorIORev.RevFollower;
import frc.lib.io.motor.MotorIORevSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


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

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final boolean tuningMode = false;
    public static final boolean disableHAL = false;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Distance STARTING_POSE_DRIVE_TOLERANCE = Inches.of(3.0); // For auto
    public static final Angle STARTING_POSE_ROT_TOLERANCE_DEGREES = Degrees.of(5.0);

    public static final Distance FULL_ROBOT_WIDTH = Inches.of(27.0 + 3.25);
    public static final Distance FULL_ROBOT_LENGTH = Inches.of(27.0 + 3.25);
    public static final Distance BUMPER_HEIGHT = Inches.of(4.0);

    public enum RobotType {
        COMP,
        ALPHA,
        NONE
    }

  public static class BuildConstants {
        public static final String PROJECT_NAME = "MotorTrainerBoard";
        public static final String VERSION = "Teaching/Testing";
        public static final String GIT_REVISION = "1.2.0";
        public static final String GIT_BRANCH = "main";
        public static final String BUILD_DATE = "2025-08-23 11:01 AM EST";
        public static final int DIRTY = 0;
  }

    public class Ports {
        /*
        * LIST OF CHANNEL AND CAN IDS
        */

        public static final Device.CAN REV_pdh = new Device.CAN(50, "rio");

        public static final Device.CAN SingleMotor = new Device.CAN(14, "rio");

        public static final Device.CAN DoubleMotorMain = new Device.CAN(10, "rio");
        public static final Device.CAN DoubleMotorFollower = new Device.CAN(11, "rio");

    }
    
    public class SingleMotorConstants {
        public static final String NAME = "1_SingleMotor";
    
        public static final Angle TOLERANCE_POSITION  = Rotations.of(0.01);
        public static final AngularVelocity TOLERANCE_VELOCITY = RotationsPerSecond.of(0.01);
    
        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.2);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0.1);
        public static final Velocity<edu.wpi.first.units.AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    
        private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
        private static final double SENSOR_TO_MECHANISM = (100.0 / 1.0);
    
        public static final Translation3d OFFSET = Translation3d.kZero;
    
        public static final Angle MIN_ANGLE = Rotations.of(0.0);
        public static final Angle MAX_ANGLE = Rotations.of(1);
        public static final Angle STARTING_ANGLE = Rotations.of(0.0);
        public static final Distance WHEEL_RADIUS = Meters.of(0.05);
    
        public static final RotaryMechCharacteristics CONSTANTS =
            new RotaryMechCharacteristics(WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE, RotaryAxis.YAW);

        public static final Mass WHEEL_MASS = Kilograms.of(0.1);
        public static final DCMotor DCMOTOR = DCMotor.getNeo550(1);
        public static final MomentOfInertia MOI = KilogramSquareMeters
            .of(0.5*WHEEL_MASS.in(Kilogram)*Math.pow(WHEEL_RADIUS.in(Meter),2));
    
        public static final Setpoint DEFAULT_SETPOINT = Setpoint.STOP;

        public static final PID SLOT0_PID = new PID(10.0, 0.0, 0.0).withS(0.0); //Postion
        public static final PID SLOT1_PID = new PID(0.01, 0.0, 0.0).withV(0.511); // Velocity

        public static SparkMaxConfig getREVConfig()
        {
            SparkMaxConfig config = new SparkMaxConfig();

            config.smartCurrentLimit(60)
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            //.advanceCommutation(120.0)//Uncomment if using minion this is from SparkMaxConfig.Presets.CTRE_Minion some peole say -120 works
            
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

            config.encoder
            .positionConversionFactor(1/SENSOR_TO_MECHANISM)
            .velocityConversionFactor(1/SENSOR_TO_MECHANISM/60);
            //Uncomment if using velocity control
            // .uvwMeasurementPeriod(10)
            // .uvwAverageDepth(2);

            config.softLimit
            .forwardSoftLimit(MAX_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(MIN_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(false);

            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

                // Position slot 0
                .p(SLOT0_PID.P(), ClosedLoopSlot.kSlot0)
                .i(SLOT0_PID.I(), ClosedLoopSlot.kSlot0)
                .d(SLOT0_PID.D(), ClosedLoopSlot.kSlot0)
                .iZone(0, ClosedLoopSlot.kSlot0)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot0)
                .outputRange(-1,1, ClosedLoopSlot.kSlot0)
    
                //Velocity slot 1
                .p(SLOT1_PID.P(), ClosedLoopSlot.kSlot1)
                .i(SLOT1_PID.I(), ClosedLoopSlot.kSlot1)
                .d(SLOT1_PID.D(), ClosedLoopSlot.kSlot1)
                .iZone(0, ClosedLoopSlot.kSlot1)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1,1, ClosedLoopSlot.kSlot1)

                // Position slot 2 with motion profile
                .p(SLOT0_PID.P(), ClosedLoopSlot.kSlot2)
                .i(SLOT0_PID.I(), ClosedLoopSlot.kSlot2)
                .d(SLOT0_PID.D(), ClosedLoopSlot.kSlot2)
                .iZone(0, ClosedLoopSlot.kSlot2)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot2)
                .outputRange(-1,1, ClosedLoopSlot.kSlot2)
    
                //Velocity slot 3 with motion profile
                .p(SLOT1_PID.P(), ClosedLoopSlot.kSlot3)
                .i(SLOT1_PID.I(), ClosedLoopSlot.kSlot3)
                .d(SLOT1_PID.D(), ClosedLoopSlot.kSlot3)
                .iZone(0, ClosedLoopSlot.kSlot3)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot3)
                .outputRange(-1,1, ClosedLoopSlot.kSlot3)

                .feedForward
                .kS(SLOT0_PID.S(),ClosedLoopSlot.kSlot0)
                .kV(SLOT0_PID.V(),ClosedLoopSlot.kSlot0)
                .kA(SLOT0_PID.A(),ClosedLoopSlot.kSlot0)
                .kG(SLOT0_PID.G(),ClosedLoopSlot.kSlot0)
                .kCosRatio(0,ClosedLoopSlot.kSlot0)

                .kS(SLOT1_PID.S(),ClosedLoopSlot.kSlot1)
                .kV(SLOT1_PID.V(),ClosedLoopSlot.kSlot1)
                .kA(SLOT1_PID.A(),ClosedLoopSlot.kSlot1)

                .kS(SLOT0_PID.S(),ClosedLoopSlot.kSlot2)
                .kV(SLOT0_PID.V(),ClosedLoopSlot.kSlot2)
                .kA(SLOT0_PID.A(),ClosedLoopSlot.kSlot2)
                .kG(SLOT0_PID.G(),ClosedLoopSlot.kSlot2)
                .kCosRatio(0,ClosedLoopSlot.kSlot2)

                .kS(SLOT1_PID.S(),ClosedLoopSlot.kSlot3)
                .kV(SLOT1_PID.V(),ClosedLoopSlot.kSlot3)
                .kA(SLOT1_PID.A(),ClosedLoopSlot.kSlot3);



            config.closedLoop.maxMotion
            //Position 
                .cruiseVelocity(CRUISE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot2)
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot2)
                .allowedProfileError(TOLERANCE_POSITION.in(Rotations),ClosedLoopSlot.kSlot2)
                .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal,ClosedLoopSlot.kSlot2)
                
            //Velocity
                .cruiseVelocity(CRUISE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot3)
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot3)
                .allowedProfileError(TOLERANCE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot3);
    
            return config;
        }

    public static FlywheelMechanism<?> getSingleMotorIO()
    {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = 
                    new FlywheelMechanismReal(NAME,
                        new MotorIORev(NAME, getREVConfig(), Ports.SingleMotor));
                break;
            case SIM:
                mechanism = 
                    new FlywheelMechanismSim(
                        NAME,
                        new MotorIORevSim(NAME, getREVConfig(),Ports.SingleMotor,ROTOR_TO_SENSOR,SENSOR_TO_MECHANISM,DCMOTOR),
                        DCMOTOR, MOI, TOLERANCE_VELOCITY);
                break;
            case REPLAY:
                mechanism = 
                    new FlywheelMechanism<>(NAME, new MotorIO(){}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);

        return mechanism;
    }

    // private static final LoggedTunableNumber RAISED_SETPOINT = new LoggedTunableNumber("RAISED", 1);
    // private static final LoggedTunableNumber UNJAM_SETPOINT = new LoggedTunableNumber("TEST", -1);

        @RequiredArgsConstructor
        @SuppressWarnings("Immutable")
        @Getter
        public enum Setpoint {
            STOP(RotationsPerSecond.of(0)),
            INTAKE(RotationsPerSecond.of(0.75)),
            UNJAM(RotationsPerSecond.of(-0.25)),
            FUNBOB(RotationsPerSecond.of(0.5));

            private final AngularVelocity setpoint;
        }
    }

    public class DoubleMotorConstants {
        public static String NAME = "2_DoubleMotor";

        public static final Angle TOLERANCE_POSITION  = Rotations.of(0.01);
        public static final AngularVelocity TOLERANCE_VELOCITY = RotationsPerSecond.of(0.01);
    
        public static final Angle ANGLE_TOLERANCE = Rotations.of(0.01);
        public static final AngularVelocity ANGLE_VELOCITY_TOLERANCE =RotationsPerSecond.of(0.01);
    
        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(204);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(204);
        public static final Velocity<edu.wpi.first.units.AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    
        private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
        private static final double SENSOR_TO_MECHANISM = (204.0 / 1.0);
    
        public static final Translation3d OFFSET = Translation3d.kZero;
    
        public static final Angle MIN_ANGLE = Rotations.of(0.0);
        public static final Angle MAX_ANGLE = Rotations.of(10.0);
        public static final Angle STARTING_ANGLE = Rotations.of(0.0);
        public static final Distance WHEEL_RADIUS = Meters.of(0.5);
    
        public static final RotaryMechCharacteristics CONSTANTS =
            new RotaryMechCharacteristics(WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE, RotaryAxis.YAW);
    
        public static final Mass WHEEL_MASS = Kilograms.of(0.0625);
        public static final DCMotor DCMOTOR = DCMotor.getNEO(2);

        public static final MomentOfInertia MOI = KilogramSquareMeters
            .of(17.5);
    
        public static final Setpoint DEFAULT_SETPOINT = Setpoint.STOW;

        public static final RevFollower FOLLOWER_1 = new RevFollower(Ports.DoubleMotorFollower, true);

        public static final PID SLOT0_PID = new PID(25.0, 0.0, 0.0).withS(0.0); //Postion
        public static final PID SLOT1_PID = new PID(1.2939E-10, 0.00000015, 0.0).withV(0.0000815); // Velocity
    
        /**
         * Creates and returns the TalonFX motor controller configuration for the rotary mechanism.
         * 
         * <p>
         * This configuration includes:
         * <ul>
         * <li>Current limits to prevent motor damage and brownouts</li>
         * <li>Voltage limits for power output</li>
         * <li>Brake mode to hold position when not moving</li>
         * <li>Software limit switches to prevent mechanism damage</li>
         * <li>Gear ratios for proper position/velocity feedback</li>
         * <li>Remote CANcoder feedback for absolute positioning</li>
         * <li>PID gains for control</li>
         * </ul>
         * 
         * @return A configured TalonFXConfiguration object ready to apply to a motor controller
         */        
        public static SparkMaxConfig getREVConfig()
        {
            SparkMaxConfig config = new SparkMaxConfig();

            config.smartCurrentLimit(60)
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            //.advanceCommutation(120.0)//Uncomment if using minion this is from SparkMaxConfig.Presets.CTRE_Minion some peole say -120 works
            
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

            config.encoder
            .positionConversionFactor(1/SENSOR_TO_MECHANISM)
            .velocityConversionFactor(1/SENSOR_TO_MECHANISM/60);
            //Uncomment if using velocity control
            // .uvwMeasurementPeriod(10)
            // .uvwAverageDepth(2);

            config.softLimit
            .forwardSoftLimit(MAX_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(MIN_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(false);

            config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

                // Position slot 0
                .p(SLOT0_PID.P(), ClosedLoopSlot.kSlot0)
                .i(SLOT0_PID.I(), ClosedLoopSlot.kSlot0)
                .d(SLOT0_PID.D(), ClosedLoopSlot.kSlot0)
                .iZone(0, ClosedLoopSlot.kSlot0)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot0)
                .outputRange(-1,1, ClosedLoopSlot.kSlot0)
    
                //Velocity slot 1
                .p(SLOT1_PID.P(), ClosedLoopSlot.kSlot1)
                .i(SLOT1_PID.I(), ClosedLoopSlot.kSlot1)
                .d(SLOT1_PID.D(), ClosedLoopSlot.kSlot1)
                .iZone(0, ClosedLoopSlot.kSlot1)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1,1, ClosedLoopSlot.kSlot1)

                // Position slot 2 with motion profile
                .p(SLOT0_PID.P(), ClosedLoopSlot.kSlot2)
                .i(SLOT0_PID.I(), ClosedLoopSlot.kSlot2)
                .d(SLOT0_PID.D(), ClosedLoopSlot.kSlot2)
                .iZone(0, ClosedLoopSlot.kSlot2)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot2)
                .outputRange(-1,1, ClosedLoopSlot.kSlot2)
    
                //Velocity slot 3 with motion profile
                .p(SLOT1_PID.P(), ClosedLoopSlot.kSlot3)
                .i(SLOT1_PID.I(), ClosedLoopSlot.kSlot3)
                .d(SLOT1_PID.D(), ClosedLoopSlot.kSlot3)
                .iZone(0, ClosedLoopSlot.kSlot3)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot3)
                .outputRange(-1,1, ClosedLoopSlot.kSlot3)

                .feedForward
                .kS(SLOT0_PID.S(),ClosedLoopSlot.kSlot0)
                .kV(SLOT0_PID.V(),ClosedLoopSlot.kSlot0)
                .kA(SLOT0_PID.A(),ClosedLoopSlot.kSlot0)
                .kG(SLOT0_PID.G(),ClosedLoopSlot.kSlot0)
                .kCosRatio(0,ClosedLoopSlot.kSlot0)

                .kS(SLOT1_PID.S(),ClosedLoopSlot.kSlot1)
                .kV(SLOT1_PID.V(),ClosedLoopSlot.kSlot1)
                .kA(SLOT1_PID.A(),ClosedLoopSlot.kSlot1)

                .kS(SLOT0_PID.S(),ClosedLoopSlot.kSlot2)
                .kV(SLOT0_PID.V(),ClosedLoopSlot.kSlot2)
                .kA(SLOT0_PID.A(),ClosedLoopSlot.kSlot2)
                .kG(SLOT0_PID.G(),ClosedLoopSlot.kSlot2)
                .kCosRatio(0,ClosedLoopSlot.kSlot2)

                .kS(SLOT1_PID.S(),ClosedLoopSlot.kSlot3)
                .kV(SLOT1_PID.V(),ClosedLoopSlot.kSlot3)
                .kA(SLOT1_PID.A(),ClosedLoopSlot.kSlot3);



            config.closedLoop.maxMotion
            //Position 
                .cruiseVelocity(CRUISE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot2)
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot2)
                .allowedProfileError(TOLERANCE_POSITION.in(Rotations),ClosedLoopSlot.kSlot2)
                .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal,ClosedLoopSlot.kSlot2)
                
            //Velocity
                .cruiseVelocity(CRUISE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot3)
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot3)
                .allowedProfileError(TOLERANCE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot3);
    
            return config;
        }
        public static FlywheelMechanism<?> getDoubleMotorIO()
    {
        FlywheelMechanism<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism = 
                    new FlywheelMechanismReal(NAME,
                        new MotorIORev(NAME, getREVConfig(), Ports.DoubleMotorMain, FOLLOWER_1));
                break;
            case SIM:
                mechanism = 
                    new FlywheelMechanismSim(
                        NAME,
                        new MotorIORevSim(NAME, getREVConfig(),Ports.DoubleMotorMain,ROTOR_TO_SENSOR,SENSOR_TO_MECHANISM,DCMOTOR, FOLLOWER_1),
                        DCMOTOR, MOI, TOLERANCE_VELOCITY);
                break;
            case REPLAY:
                mechanism = 
                    new FlywheelMechanism<>(NAME, new MotorIO(){}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);

        return mechanism;
    }


    // private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    // private static final LoggedTunableNumber RAISED_SETPOINT = new LoggedTunableNumber("RAISED", 12.5);

        @RequiredArgsConstructor
        @SuppressWarnings("Immutable")
        @Getter
        public enum Setpoint {
            STOW(Rotations.of(0)),
            RAISED(Rotations.of(1.0)),
            FUNBOB(Rotations.of(.5));        
            
            private final Angle setpoint;
        }
    }

    public class IntakeConstants {
        public static final String NAME = "3_Intake";



        @RequiredArgsConstructor
        @SuppressWarnings("Immutable")
        @Getter
        public enum IntakeOptions {
                STOP(SingleMotorConstants.Setpoint.STOP,DoubleMotorConstants.Setpoint.STOW),
                INTAKE(SingleMotorConstants.Setpoint.INTAKE,DoubleMotorConstants.Setpoint.RAISED),
                UNJAM(SingleMotorConstants.Setpoint.UNJAM,DoubleMotorConstants.Setpoint.RAISED),
                MIDDLE(SingleMotorConstants.Setpoint.FUNBOB,DoubleMotorConstants.Setpoint.FUNBOB);

            //   STOP(Rotations.of(0), RotationsPerSecond.of(0)),
            //   INTAKE(Rotations.of(0), RotationsPerSecond.of(1)),
            //   UNJAM(Rotations.of(0), RotationsPerSecond.of(-0.25)),
            //   FUNBOB(Rotations.of(0), RotationsPerSecond.of(0.5));


            private final SingleMotorConstants.Setpoint wheelsspeed;
            private final DoubleMotorConstants.Setpoint pivotangle;
        }
    }

    public class MotorConstants{

        public static final int LEADER_ID = 1;
    }
}