package frc.robot.lib.Motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * MotorBase
 * 
 * <p>Abstract base class representing a motor with standard telemetry and control methods.
 * Subclasses must implement all abstract methods to provide specific hardware behavior.
 * 
 * <p>Features:
 * <p>- SysIDRoutine creation and run methods
 * <p>- Hold protection methods for the motor when observing both tempurature and amps
 * <p>- Logs values from motor 
 * 
 * <p>Hardware:
 * <p>- Designed to be used with any motors and their encoders
 * 
 * <p>Reference: 
 * <p>- SysID: https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/system-identification/creating-routine.html
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public abstract class FollowerMotorBase extends SubsystemBase {

    protected final String loggerBase;

    //=========== Constructor ===========
    public FollowerMotorBase(String loggerBase) {
        this.loggerBase = loggerBase;
    }

    //=========== Logging ===========
    /**
     * Published the following values to the logger using loggerBase
     * - Motor: Position, Velocity, Amps, Volts, and Tempuratur
     * - Mechanisum(calculated using gearRatio and wheelRadius): Position, Velocity, and wheelRadius
     * Measurment will be given in degress, degress/minute, m/s, volts, amps, and celcuis.
     */
    public void logMotor() {
        Logger.recordOutput(loggerBase + "Motor_Position", getMotorPosition().getDegrees());
        Logger.recordOutput(loggerBase + "Motor_Velocity", getMotorVelocity().getDegrees());
        Logger.recordOutput(loggerBase + "Mech_Position", getMechanismPosition().getDegrees());
        Logger.recordOutput(loggerBase + "Mech_Velocity", getMechanismVelocity().getDegrees());
        Logger.recordOutput(loggerBase + "Mech_TipSpeed", getMechanismTipSpeed());
        Logger.recordOutput(loggerBase + "Amps", getMotorAmps());
        Logger.recordOutput(loggerBase + "Volts", getMotorVoltage());
        Logger.recordOutput(loggerBase + "Temp", getMotorTemperatureCelsius());
    }

    //=========== Abstract Methods (Getters) ===========

    //Encoder methods
    /** Gets raw motor-side position (before gear ratio) */ public abstract Rotation2d getMotorPosition();
    /** Gets raw motor-side velocity in rotations per minute (RPM) */ public abstract Rotation2d getMotorVelocity();
    /** Gets mechanism-side position (after gear ratio) */ public abstract Rotation2d getMechanismPosition();
    /** Gets mechanism-side velocity in rotations per minute (RPM) */ public abstract Rotation2d getMechanismVelocity();
    /** Gets mechanism tip speed in meters per second */ public abstract double getMechanismTipSpeed();
    /** Gets current voltage applied to the motor (volts) */ public abstract double getMotorVoltage();
    /** Gets current motor amperage (amps) */ public abstract double getMotorAmps();
    /** Gets motor temperature in degrees Celsius */ public abstract double getMotorTemperatureCelsius();

    //=========== Abstract Methods (Setters) =========== All Encoder methods
    /** Sets the neutral mode (Brake = true, Coast = false) */ public abstract void setIdealState(boolean isBrake);

}
