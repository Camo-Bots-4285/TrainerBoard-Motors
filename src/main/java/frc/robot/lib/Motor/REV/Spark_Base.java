package frc.robot.lib.Motor.REV;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.Motor.MotionProfileHelpers;
import frc.robot.lib.Motor.MotorBase;
import frc.robot.lib.Motor.MotorTypes;
import frc.robot.lib.Motor.MotionProfileHelpers.REV_MotionProfile;
import frc.robot.lib.Motor.MotorTypes.Motor_Type;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

/**
 * Spark_Base
 * 
 * <p> Is and abstract class which extends {@link frc.robot.lib.Motor.MotorBase MotorBase} it fill overide all abstract methods
 * that involve the motor logic
 * 
 * <p>Features:
 * <p>- Getters return relevent information from the motors
 * <p>- Setters use motor logic to send data/control the motor
 * 
 * <p>Hardware:
 * <p>- Can be used with both CTRE and Rev
 * 
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class Spark_Base extends MotorBase {

    /*Motor and Spark Identification Constants */
        private SparkBase motor;
        private SparkClosedLoopController spark_controller;
        private SparkBaseConfig motor_config;
        private RelativeEncoder motor_encoder;
        
        private final int motorID;
        private final double gearRatio;
        private final double wheelRadius;
        private final IdleMode idleMode;
        private final boolean inverted;
        private final Motor_Type motorType;
        private final REV_MotionProfile motionProfile;
    
    
    /*Constuctor to initialize values */
    public Spark_Base(
        String loggerBase,//Bring in the name of the motor so error can speciffy
        int motorID,
        double gearRatio,
        double wheelRadius,
        boolean Idle_Brake, 
        boolean inverted,
        Motor_Type motorType,
        REV_MotionProfile motionProfile,//Holds motor PID, FF, and motion Constrants
        boolean isSparkFlex
      ) {
        //Pass name of motor to the MotorBase class
        super(loggerBase);
    
      /*Set value to import or set values ot defualt if null */
        this.motorID = motorID;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.idleMode = Idle_Brake ? IdleMode.kBrake : IdleMode.kCoast;
        this.inverted=inverted;
        this.motorType = (motorType != null) ? motorType : new MotorTypes.REV_Defualt();
        this.motionProfile = (motionProfile != null) ? motionProfile : MotionProfileHelpers.getDisabledREVProfile();

    
        configureMotor(isSparkFlex);
      }
    
      public void configureMotor(boolean isSparkFlex) {
        // Create specific motor and config objects
        if (isSparkFlex) {
            SparkFlex flex = new SparkFlex(motorID, MotorType.kBrushless);
            SparkFlexConfig config = new SparkFlexConfig();
            motor = flex;
            motor_config = config;
        } else {
            SparkMax max = new SparkMax(motorID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            motor = max;
            motor_config = config;
        }
    
        // Common controller/encoder references
        spark_controller = motor.getClosedLoopController();
    motor_encoder = motor.getEncoder();

    // Apply shared configuration values
    motor_config
        .smartCurrentLimit(motorType.getDefaultAmpLimits()[0], motorType.getDefaultAmpLimits()[1], motorType.getDefaultAmpLimits()[2])
        .inverted(inverted)
        .idleMode(idleMode);

    // Closed-loop PID slot configuration
    motor_config.closedLoop
        // Slot 0 - Position
        .p(motionProfile.kP_Pos, ClosedLoopSlot.kSlot0)
        .i(motionProfile.kI_Pos, ClosedLoopSlot.kSlot0)
        .d(motionProfile.kD_Pos, ClosedLoopSlot.kSlot0)
        .iZone(motionProfile.iZone_Pos, ClosedLoopSlot.kSlot0)
        .iMaxAccum(motionProfile.iMaxAccum_Pos, ClosedLoopSlot.kSlot0)
        .outputRange(motionProfile.outMin_Pos, motionProfile.outMax_Pos, ClosedLoopSlot.kSlot0)

        // Slot 1 - Velocity
        .p(motionProfile.kP_Vel, ClosedLoopSlot.kSlot1)
        .i(motionProfile.kI_Vel, ClosedLoopSlot.kSlot1)
        .d(motionProfile.kD_Vel, ClosedLoopSlot.kSlot1)
        .velocityFF(motionProfile.kV_Vel, ClosedLoopSlot.kSlot1)
        .iZone(motionProfile.iZone_Vel, ClosedLoopSlot.kSlot1)
        .iMaxAccum(motionProfile.iMaxAccum_Vel, ClosedLoopSlot.kSlot1)
        .outputRange(motionProfile.outMin_Vel, motionProfile.outMax_Vel, ClosedLoopSlot.kSlot1)

        // Slot 2 - Alternate
        .pidf(motionProfile.kP_Alt, motionProfile.kI_Alt, motionProfile.kD_Alt, motionProfile.kV_Alt, ClosedLoopSlot.kSlot2)
        .iZone(motionProfile.iZone_Alt, ClosedLoopSlot.kSlot2)
        .iMaxAccum(motionProfile.iMaxAccum_Alt, ClosedLoopSlot.kSlot2)
        .outputRange(motionProfile.outMin_Alt, motionProfile.outMax_Alt, ClosedLoopSlot.kSlot2);

    // Motion constraints for slots
    motor_config.closedLoop.maxMotion
        .maxVelocity(motionProfile.cruiseRPM, ClosedLoopSlot.kSlot0)
        .maxAcceleration(motionProfile.accelRPMPerSec, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(motionProfile.allowedErrorRotations, ClosedLoopSlot.kSlot0)

        .maxVelocity(motionProfile.cruiseRPM, ClosedLoopSlot.kSlot1)
        .maxAcceleration(motionProfile.accelRPMPerSec, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(motionProfile.allowedErrorRotations, ClosedLoopSlot.kSlot1);

    // Encoder update rate
    motor_config.signals.primaryEncoderPositionPeriodMs(20);

    // Apply configuration
    motor.configure(motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

      

    // ========== Getters ==========
    @Override
    public Motor_Type getMotorType(){
        return motorType;
    }

    @Override
    public Rotation2d getMotorPosition() {
        return Rotation2d.fromRotations(motor_encoder.getPosition());
    }

    @Override
    public Rotation2d getMotorVelocity() {
        return Rotation2d.fromRotations(motor_encoder.getVelocity());
    }

    @Override
    public Rotation2d getMechanismPosition() {
        return Rotation2d.fromRotations(motor_encoder.getPosition() / gearRatio);
    }

    @Override
    public Rotation2d getMechanismVelocity() {
        return Rotation2d.fromRotations(motor_encoder.getVelocity() / gearRatio);
    }

    @Override
    public double getMechanismTipSpeed() {
        return getMechanismVelocity().getRadians() * wheelRadius;
    }

    @Override
    public double getMotorVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public double getMotorAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getMotorTemperatureCelsius() {
        return motor.getMotorTemperature();
    }

    // ========== Setters ==========

    @Override
    public void setCurrentMechanismPosition(double rotations) {
        motor_encoder.setPosition(rotations * gearRatio);
        Logger.recordOutput(loggerBase + "targetMechPosition", rotations);
    }

    @Override
    public void setCurrentMotorPosition(double rotations) {
        motor_encoder.setPosition(rotations);
        Logger.recordOutput(loggerBase + "targetMotorPosition", rotations);
    }

    @Override
    public void setTargetMotorPosition(double rotations) {
        spark_controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        Logger.recordOutput(loggerBase + "targetMotorPosition", rotations);
    }

    @Override
    public void setTargetMotorVelocity(double rpm) {
        spark_controller.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        Logger.recordOutput(loggerBase + "targetMotorVelocity", rpm);
    }

    @Override
    public void setTargetMotorVoltage(double volts) {
        spark_controller.setReference(volts, ControlType.kVoltage, ClosedLoopSlot.kSlot2);
        Logger.recordOutput(loggerBase + "targetMotorVoltage", volts);
    }

    @Override
    public void setMotorPercent(double percent) {
        motor.set(percent);
        Logger.recordOutput(loggerBase + "motorPercent", percent);
    }

    @Override
    public void setMotorStop() {
        motor.stopMotor();
        Logger.recordOutput(loggerBase + "motorStop", "Stopped");
    }

    @Override
    public void setTargetMechanismPosition(double rotations) {
        setTargetMotorPosition(rotations * gearRatio);
        Logger.recordOutput(loggerBase + "targetMechanismPosition", rotations);
    }

    @Override
    public void setTargetMechanismVelocity(double rpm) {
        setTargetMotorVelocity(rpm * gearRatio);
        Logger.recordOutput(loggerBase + "targetMechanismVelocity", rpm);
    }

    @Override
    public void setTargetMechanismTipSpeed(double metersPerSecond) {
        double targetRPM = metersPerSecond / (2 * Math.PI * wheelRadius) * gearRatio * 60;
        setTargetMechanismVelocity(targetRPM);
        Logger.recordOutput(loggerBase + "targetMechanismTipSpeed", metersPerSecond);
    }


    @Override
    public void setIdealState(boolean isBrake) {
        SparkBaseConfig config = newEmptyConfig();

        IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
        
        config.idleMode(mode);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    //====== NonAbstract ===== 
    // These methods are to differnt between encoder to have generaic input
    private SparkBaseConfig newEmptyConfig() {
        return (motor instanceof SparkFlex)
            ? new SparkFlexConfig()
            : new SparkMaxConfig();
    }
    

    public ClosedLoopSlot getSlot(int slotNumber){

        if (slotNumber == 0) {
            return ClosedLoopSlot.kSlot0;
        } else if (slotNumber == 1) {
            return ClosedLoopSlot.kSlot1;
        } else if (slotNumber == 2) {
            return ClosedLoopSlot.kSlot2;
        } else if (slotNumber == 3) {
            return ClosedLoopSlot.kSlot3;
        } else {
            // Optional: handle invalid input
            System.out.println("Invalid slot number: " + slotNumber);
            return ClosedLoopSlot.kSlot0;
        }

    }

    public void setTunerConstants(double p, double i, double d, double ff, double iZone, double iMaxAccum, double minOutput, double maxOutput, int slotNumber) {
        SparkBaseConfig config = newEmptyConfig();
    
        ClosedLoopSlot slot = getSlot(slotNumber);
    
        config.closedLoop
            .p(p, slot)
            .i(i, slot)
            .d(d, slot)
            .velocityFF(ff, slot)
            .iZone(iZone, slot)
            .iMaxAccum(iMaxAccum, slot)
            .outputRange(minOutput, maxOutput, slot);
    
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setMotionConstraints(double maxVelocity, double maxAcceleration, double allowedError, int slotNumber) {
        SparkBaseConfig config = newEmptyConfig();

        ClosedLoopSlot Slot=getSlot(slotNumber);

        config.closedLoop.maxMotion
            .maxVelocity(maxVelocity,Slot)
            .maxAcceleration(maxAcceleration,Slot)
            .allowedClosedLoopError(allowedError,Slot);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void setAmpLimits(int stallLimit, int freeLimit, int limitRpm) {
        SparkBaseConfig config = newEmptyConfig();
    
        config.smartCurrentLimit(stallLimit, freeLimit, limitRpm);
    
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SparkBaseConfig getConfig(){
        return motor_config;
    }
    

}
