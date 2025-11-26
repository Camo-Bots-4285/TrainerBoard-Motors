package frc.robot;

import java.util.Optional;

import frc.lib.W8.util.Device;
import frc.lib.W8.util.Device.CAN;
import frc.lib.W8.util.LoggedTunableNumber;
import frc.lib.W8.io.motor.MotorIO;
import frc.lib.W8.io.motor.MotorIORev;
import frc.lib.W8.io.motor.MotorIORev.RevFollower;
import frc.lib.W8.io.motor.MotorIORevSim;
import frc.lib.W8.io.motor.MotorIOSim;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.W8.mechanisms.rotary.*;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
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

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

  public static class BuildConstants {
        public static final String PROJECT_NAME = "MotorTrainerBoard";
        public static final String VERSION = "Teaching/Testing";
        public static final String GIT_REVISION = "1.2.0";
        public static final String GIT_BRANCH = "main";
        public static final String BUILD_DATE = "2025-08-23 11:01 AM EST";
        public static final int DIRTY = 0;
  }

  public enum RobotType {
    COMP,
    ALPHA,
    NONE
}

    public class Ports {
        /*
        * LIST OF CHANNEL AND CAN IDS
        */

        public static final Device.CAN REV_pdh = new CAN(50, "rio");

        public static final Device.CAN SingleMotor = new CAN(14, "rio");

        public static final Device.CAN DoubleMotorMain = new CAN(10, "rio");
        public static final Device.CAN DoubleMotorFollower = new CAN(11, "rio");

    }
    
    public class SingleMotorConstants {
        public static final String NAME = "1_SingleMotor";

        public static final boolean isFlex = false;
    
        public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(0.01);
    
        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.2);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0.1);
        public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    
        private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
        private static final double SENSOR_TO_MECHANISM = (100.0 / 1.0);
    
        public static final Translation3d OFFSET = Translation3d.kZero;
    
        public static final Angle MIN_ANGLE = Rotations.of(0.0);
        public static final Angle MAX_ANGLE = Rotations.of(1);
        public static final Angle STARTING_ANGLE = Rotations.of(0.0);
        public static final Distance ARM_LENGTH = Meters.of(0.05);
    
        public static final RotaryMechCharacteristics CONSTANTS =
            new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);
    
        public static final Mass ARM_MASS = Kilograms.of(0.01);
        public static final DCMotor DCMOTOR = DCMotor.getNeo550(1);
        public static final MomentOfInertia MOI = KilogramSquareMeters
            .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));
    
        public static final Setpoint DEFAULT_SETPOINT = Setpoint.STOP;

        public static SparkMaxConfig getREVConfig()
        {
            SparkMaxConfig config = new SparkMaxConfig();
    
            config.smartCurrentLimit(
                20,
                35,
                60
            )
            .voltageCompensation(12.0)
            .idleMode(false ? IdleMode.kBrake : IdleMode.kCoast)
            .inverted(false)
            .signals.primaryEncoderPositionPeriodMs(20);

            //Place gear ratio in this
            config.encoder
            .positionConversionFactor(1/SENSOR_TO_MECHANISM)
            .velocityConversionFactor(1/SENSOR_TO_MECHANISM/60); 

            config.softLimit
            .forwardSoftLimit(MAX_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(MIN_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(false);
    
            config.closedLoop
                // Position slot 0
                .p(10.0, ClosedLoopSlot.kSlot0)
                .i(0.0, ClosedLoopSlot.kSlot0)
                .d(0.0, ClosedLoopSlot.kSlot0)
                .velocityFF(0.0, ClosedLoopSlot.kSlot0)
                .iZone(0, ClosedLoopSlot.kSlot0)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot0)
                .outputRange(-1,1, ClosedLoopSlot.kSlot0)
    
                //Velocity slot 1
                .p(0.01, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0.0, ClosedLoopSlot.kSlot1)
                .velocityFF( 0, ClosedLoopSlot.kSlot1)
                .iZone(0, ClosedLoopSlot.kSlot1)
                .iMaxAccum(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1,1, ClosedLoopSlot.kSlot1)
    
            .maxMotion
            //Position
                .maxVelocity(ACCELERATION.in(RotationsPerSecondPerSecond)*ROTOR_TO_SENSOR*SENSOR_TO_MECHANISM,ClosedLoopSlot.kSlot0)
                .maxAcceleration(CRUISE_VELOCITY.in(RotationsPerSecond)*4*ROTOR_TO_SENSOR*SENSOR_TO_MECHANISM,ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(TOLERANCE.in(RotationsPerSecond),ClosedLoopSlot.kSlot0)
                
            //Velocity
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(TOLERANCE.in(RotationsPerSecond),ClosedLoopSlot.kSlot1);
                

    

    
    
            return config;
        }
    

        public static FlywheelMechanismReal getReal()
        {
            MotorIO io = new MotorIORev(NAME, Ports.SingleMotor, isFlex, getREVConfig());
    
            return new FlywheelMechanismReal(io);
        }
    
        public static FlywheelMechanism getSim()
        {
            MotorIOSim io = new MotorIORevSim(
                NAME,
                Ports.SingleMotor,
                isFlex,
                ROTOR_TO_SENSOR,
                SENSOR_TO_MECHANISM,
                DCMOTOR,
                getREVConfig());
    
                return new FlywheelMechanismSim(io,
                DCMOTOR, MOI, TOLERANCE);
        }
    
        public static FlywheelMechanism getReplay()
        {
            return new FlywheelMechanism() {};
        }

    // private static final LoggedTunableNumber RAISED_SETPOINT = new LoggedTunableNumber("RAISED", 1);
    // private static final LoggedTunableNumber UNJAM_SETPOINT = new LoggedTunableNumber("TEST", -1);

        @RequiredArgsConstructor
        @SuppressWarnings("Immutable")
        @Getter
        public enum Setpoint {
            STOP(RotationsPerSecond.of(0)),
            INTAKE(RotationsPerSecond.of(0.001)),
            UNJAM(RotationsPerSecond.of(-0.25));

            private final AngularVelocity setpoint;
        }
    }

    public class DoubleMotorConstants {
        public static String NAME = "2_DoubleMotor";

        public static boolean isFlex = false;
    
        public static final AngularVelocity TOLERANCE =RotationsPerSecond.of(0.01);
    
        public static final AngularVelocity CRUISE_VELOCITY = Units.RotationsPerSecond.of(204);
        public static final AngularAcceleration ACCELERATION = Units.RotationsPerSecondPerSecond.of(204);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    
        private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
        private static final double SENSOR_TO_MECHANISM = (204.0 / 1.0);
    
        public static final Translation3d OFFSET = Translation3d.kZero;
    
        public static final Angle MIN_ANGLE = Rotations.of(0.0);
        public static final Angle MAX_ANGLE = Rotations.of(10.0);
        public static final Angle STARTING_ANGLE = Rotations.of(0.0);
        public static final Distance WHEEL_RADIUS = Meters.of(0.5);
    
        public static final RotaryMechCharacteristics CONSTANTS =
            new RotaryMechCharacteristics(OFFSET, WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);
    
        public static final Mass WHEEL_MASS = Kilograms.of(5.0);
        public static final DCMotor DCMOTOR = DCMotor.getNEO(2);

        public static final MomentOfInertia MOI = KilogramSquareMeters
            .of(0.5*WHEEL_MASS.in(Kilogram)*Math.pow(WHEEL_RADIUS.in(Meter), 2));

    
        public static final Setpoint DEFAULT_SETPOINT = Setpoint.STOW;

        public static final RevFollower FOLLOWER_1 = new RevFollower(Ports.DoubleMotorFollower, true);
    
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
    
            config.smartCurrentLimit(
                20,
                35,
                60
            )
            .voltageCompensation(12.0)
            .idleMode(false ? IdleMode.kBrake : IdleMode.kCoast)
            .inverted(false)
            .signals.primaryEncoderPositionPeriodMs(20);


    
            config.closedLoop
                // Position slot 0
                .pid(5, 0, 0.002,ClosedLoopSlot.kSlot0)
    
                //Velocity slot 1
                .p(1.2939E-10, ClosedLoopSlot.kSlot1)
                .i(0.00000015, ClosedLoopSlot.kSlot1)
                .d(0.0, ClosedLoopSlot.kSlot1)
                .velocityFF( 0.0000815, ClosedLoopSlot.kSlot1)
                .iZone(75.0, ClosedLoopSlot.kSlot1)
                .iMaxAccum(0.003, ClosedLoopSlot.kSlot1)
                .outputRange(-1,1, ClosedLoopSlot.kSlot1)
    
            .maxMotion
            //Global Motion control
                .maxVelocity(CRUISE_VELOCITY.in(RotationsPerSecond),ClosedLoopSlot.kSlot0)
                .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond),ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(TOLERANCE.in(RotationsPerSecond),ClosedLoopSlot.kSlot0);
    
            config.softLimit
            .forwardSoftLimit(MAX_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(MIN_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(true);

            //Place gear ratio in this
            config.encoder
            .positionConversionFactor(1/SENSOR_TO_MECHANISM)
            .velocityConversionFactor(1/SENSOR_TO_MECHANISM/60); 

    
            return config;
        }
    
        /**
         * Creates the real robot implementation of the rotary mechanism.
         * 
         * <p>
         * This method instantiates the actual hardware objects (TalonFX motors and CANcoder) that will
         * be used when running on a real robot.
         * 
         * @return A RotaryMechanismReal object configured with real hardware
         */
        public static FlywheelMechanismReal getReal()
        {
            MotorIO io = new MotorIORev(NAME, Ports.DoubleMotorMain, isFlex, getREVConfig(),FOLLOWER_1);
    
            return new FlywheelMechanismReal(io);
        }
    
        /**
         * Creates the simulation implementation of the rotary mechanism.
         * 
         * <p>
         * This method creates a physics-based simulation of the mechanism using WPILib's simulation
         * classes. It models the motor, moment of inertia, and other physical properties to provide
         * realistic behavior in simulation.
         * 
         * @return A RotaryMechanismSim object configured for physics simulation
         */
        public static FlywheelMechanism getSim()
        {
            MotorIOSim io = new MotorIORevSim(
                NAME,
                Ports.DoubleMotorMain,
                isFlex,
                ROTOR_TO_SENSOR,
                SENSOR_TO_MECHANISM,
                DCMOTOR,
                getREVConfig(),
                FOLLOWER_1
                );
    
                return new FlywheelMechanismSim(io,
                DCMOTOR, MOI, TOLERANCE);
        }
    
        /**
         * Creates the log replay implementation of the rotary mechanism.
         * 
         * <p>
         * This is used with AdvantageKit's log replay feature, which allows you to replay logged data
         * and debug robot code without having the actual robot or running simulation.
         * 
         * @return A RotaryMechanism object for log replay
         */
        public static FlywheelMechanism getReplay()
        {
            return new FlywheelMechanism() {};
        }

    // private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    // private static final LoggedTunableNumber RAISED_SETPOINT = new LoggedTunableNumber("RAISED", 12.5);

        @RequiredArgsConstructor
        @SuppressWarnings("Immutable")
        @Getter
        public enum Setpoint {
            STOW(Rotations.of(0)),
            RAISED(Rotations.of(1));

            private final Angle setpoint;
        }
    }

}