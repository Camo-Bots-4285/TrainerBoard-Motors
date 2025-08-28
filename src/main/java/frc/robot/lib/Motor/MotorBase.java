package frc.robot.lib.Motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.Motor.MotorTypes.Motor_Type;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Timer;


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
public abstract class MotorBase extends SubsystemBase {

    protected final String loggerBase;

    //=========== Instance variables ===========
    private int excessCount = 0;
    private int currentIndex = 0;
    private double[] currentReadings;
    private double emergencyStopTime = -1;

    //=========== Constructor ===========
    public MotorBase(String loggerBase) {
        this.loggerBase = loggerBase;
        currentReadings = new double[getMotorType().getNumLoops()];
        for (int i = 0; i < getMotorType().getNumLoops(); i++) currentReadings[i] = 0.0;
    }

    //=========== Protection logic ===========
    /**
     * Provides a way to track if the motor has been consistenly passing a safe limit
     * @return True if motor is believed = to be damaging itself by drawing to much amps
     */
    public boolean needEmergencyStop() {
        double current = getMotorAmps();
        currentReadings[currentIndex] = current;
        if (current > getMotorType().getEmergencyStopCurrent()) excessCount++;
        currentIndex = (currentIndex + 1) % getMotorType().getNumLoops();
        return excessCount >= getMotorType().getRequiredLoopsToShutOff();
    }

    /**
     * Provides a way to track if the motor has a greater temperature then what is considered safe
     * @return True if motor is believed to being damaging by too high of temperature
     */
    public boolean motorOverheating() {
        return getMotorTemperatureCelsius() >= getMotorType().getOverHeatTemp();
    }

    /**
     * Stops the motor if it has been drawing too much amps and gives warning it is getting too hot
     */
    public void runMotorProtection() {
        double currentTime = getCurrentTimeSeconds();
        if (needEmergencyStop()) {
            emergencyStopTime = currentTime;
            setMotorStop();
            System.err.println("WARNING " + loggerBase + ": Emergency Stopped due to high current");
        } else if (currentTime - emergencyStopTime < getMotorType().getEmergencyStopCooldownDuration()) {
            setMotorStop();
            System.err.println("WARNING " + loggerBase + ": Emergency Stop cooldown active");
        } else {
            excessCount = 0;
        }
        runTemperatureWarning();
    }

    /**
     * Sends warning to driverstation if a motor is overheating
     */
    public void runTemperatureWarning() {
        if (motorOverheating()) {
            System.err.println("WARNING " + loggerBase + ": Motor is overheating");
        }
    }


    //============ SysID ==============
    /**
    * Method that retruns a SysIdRoutine based on the following parameters while using the loggingBase of the motor
    * @param rampRate the amount of volt Dynamic will increase per second. Default ramp rate (1 V/s).
    * @param voltage the amount of volt used in Quasistatic (constant voltage). Default 7 volts
    * @param time the amount of time the robot will run the sysID. Default 10 sec
    * @return SysIdRoutine that can be passed though to run  command sysIdQuasistatic and sysIdDynamic
    */
    public SysIdRoutine getSysIdRoutine(double rampRate, double voltage,double time) {
        //new SysIdRoutine.Conf
              /* SysId routine for characterizing a motion. This is used to find PID gains for a motor. */
                   SysIdRoutine sysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                        Volts.of(rampRate).per(Second),        // Use default ramp rate (1 V/s)
                        Volts.of(voltage), // Reduce dynamic step voltage to 4 V to prevent brownout
                        Seconds.of(time),        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> Logger.recordOutput(loggerBase + "SysId_State", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                        output -> 
                            /* output is actually radians per second, but SysId only supports "volts" */
                            setTargetMotorVoltage(output.in(Volts)),
                        null,
                        this
                    )
                );

        return sysIdRoutine;
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

    //=========== Concrete helper ===========
    public double getCurrentTimeSeconds() {
        return Timer.getFPGATimestamp();
    }

    //=========== Abstract Methods (Getters) ===========

    //Encoder methods
    public abstract Motor_Type getMotorType();
    /** Gets raw motor-side position (before gear ratio) */ public abstract Rotation2d getMotorPosition();
    /** Gets raw motor-side velocity in rotations per minute (RPM) */ public abstract Rotation2d getMotorVelocity();
    /** Gets mechanism-side position (after gear ratio) */ public abstract Rotation2d getMechanismPosition();
    /** Gets mechanism-side velocity in rotations per minute (RPM) */ public abstract Rotation2d getMechanismVelocity();
    /** Gets mechanism tip speed in meters per second */ public abstract double getMechanismTipSpeed();
    /** Gets current voltage applied to the motor (volts) */ public abstract double getMotorVoltage();
    /** Gets current motor amperage (amps) */ public abstract double getMotorAmps();
    /** Gets motor temperature in degrees Celsius */ public abstract double getMotorTemperatureCelsius();

    //=========== Abstract Methods (Setters) =========== All Encoder methods
    /** Sets the current mechanism position (after gear ratio) in rotations */ public abstract void setCurrentMechanismPosition(double rotations);
    /** Sets the current motor position (before gear ratio) in rotations */ public abstract void setCurrentMotorPosition(double rotations);
    /** Sets the target motor position (before gear ratio) in rotations */ public abstract void setTargetMotorPosition(double rotations);
    /** Sets the target motor velocity (before gear ratio) in RPM */ public abstract void setTargetMotorVelocity(double rpm);
    /** Sets the raw output voltage to the motor */ public abstract void setTargetMotorVoltage(double volts);
    /** Sets motor output as a percent [-1.0 to 1.0] */ public abstract void setMotorPercent(double percent);
    /** Immediately stops the motor */ public abstract void setMotorStop();
    /** Sets the target mechanism position (after gear ratio) in rotations */ public abstract void setTargetMechanismPosition(double rotations);
    /** Sets the target mechanism velocity (after gear ratio) in RPM */ public abstract void setTargetMechanismVelocity(double rpm);
    /** Sets the tip speed of the mechanism in meters per second */ public abstract void setTargetMechanismTipSpeed(double metersPerSecond);
    /** Sets the neutral mode (Brake = true, Coast = false) */ public abstract void setIdealState(boolean isBrake);

}
