// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.revRotary;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import java.util.Optional;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.W8.io.motor.MotorIO;
import frc.lib.W8.io.motor.MotorIORev;
import frc.lib.W8.io.motor.MotorIORev.RevFollowerFollower;
import frc.lib.W8.io.motor.MotorIORevSim;
import frc.lib.W8.io.motor.MotorIORevSim.*;
import frc.lib.W8.io.motor.MotorIOSim;
import frc.lib.W8.mechanisms.rotary.*;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.robot.Ports;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Add your docs here. */
public class RevRotarySubsystemConstants {
    public static String NAME = "REVRotary";

    public static final Angle TOLERANCE =Rotations.of(0.05);

    public static final AngularVelocity CRUISE_VELOCITY =
        Units.RotationsPerSecond.of(0.05);
    public static final AngularAcceleration ACCELERATION =Units.RotationsPerSecondPerSecond.of(0.5);
     
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = (72.0 / 8.0);

    public static final Translation3d OFFSET = Translation3d.kZero;

    public static final Angle MIN_ANGLE = Rotations.of(0.0);
    public static final Angle MAX_ANGLE = Rotations.of(0.5);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final Distance ARM_LENGTH = Meters.of(0.05);

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final Mass ARM_MASS = Kilograms.of(0.01);
    public static final DCMotor DCMOTOR = DCMotor.getNeoVortex(2);
    public static final MomentOfInertia MOI = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    private static final Angle ENCODER_OFFSET = Rotations.of(0.0);

    public static final RevRotarySubsystem.Setpoint DEFAULT_SETPOINT =
        RevRotarySubsystem.Setpoint.STOW;



    // Positional PID
    private static ClosedLoopConfig SLOT0CONFIG = new ClosedLoopConfig()
        .pid(1, 0, 0.0, ClosedLoopSlot.kSlot0);

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
    public static SparkFlexConfig getREVConfig()
    {
        SparkFlexConfig config = new SparkFlexConfig();

        config.smartCurrentLimit(
            20,
            35,
            60
        )
        .voltageCompensation(12.0)
        .idleMode(true ? IdleMode.kBrake : IdleMode.kCoast)
        .inverted(false)
        .signals.primaryEncoderPositionPeriodMs(20);

        config.closedLoop
            // Position slot 0
            .pid(0.1, 0, 0.001,ClosedLoopSlot.kSlot0)

            .p(0.1, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0.0, ClosedLoopSlot.kSlot1)
            .velocityFF(0.0, ClosedLoopSlot.kSlot1)
            .iZone(0, ClosedLoopSlot.kSlot1)
            .iMaxAccum(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1,1, ClosedLoopSlot.kSlot1)

        .maxMotion
        //Global Motion control
        .maxVelocity(CRUISE_VELOCITY.in(RotationsPerSecond), ClosedLoopSlot.kSlot0)
        .maxAcceleration(ACCELERATION.in(RotationsPerSecondPerSecond), ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0)

            .maxVelocity(5,ClosedLoopSlot.kSlot1)
            .maxAcceleration(2,ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(0.01,ClosedLoopSlot.kSlot1);
        

        config.softLimit
        .forwardSoftLimit(MAX_ANGLE.in(Rotations))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(MIN_ANGLE.in(Rotations))
        .reverseSoftLimitEnabled(true);

        //Place gear ratio in this
        config.encoder
        .positionConversionFactor(SENSOR_TO_MECHANISM)
        .velocityConversionFactor(SENSOR_TO_MECHANISM/60); 


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
    public static RotaryMechanismReal getReal()
    {
        MotorIO io = new MotorIORev(NAME, Ports.RotarySubsystemMotorMain, true, getREVConfig(), new RevFollowerFollower(Ports.RotarySubsystemMotorFollower, true));

        return new RotaryMechanismReal(io, CONSTANTS, null);
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
    public static RotaryMechanismSim getSim()
    {
        MotorIOSim io = new MotorIORevSim(
            NAME,
            Ports.RotarySubsystemMotorMain,
            true,
            ROTOR_TO_SENSOR,
            SENSOR_TO_MECHANISM,
            DCMOTOR,
            getREVConfig());

        return new RotaryMechanismSim(
            io,
            DCMOTOR,
            MOI,
            false,
            CONSTANTS,
            Optional.empty());
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
    public static RotaryMechanism getReplay()
    {
        return new RotaryMechanism(NAME, CONSTANTS) {};
    }
}
