package frc.robot.lib.Motor.CTRE;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.Motor.MotionProfileHelpers;
import frc.robot.lib.Motor.MotorBase;
import frc.robot.lib.Motor.MotorTypes;
import frc.robot.lib.Motor.MotionProfileHelpers.CTRE_MotionProfile;
import frc.robot.lib.Motor.MotorTypes.Motor_Type;

import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Talon extends MotorBase {

    private final Motor_Type motorType;
    
    /*Motor and Spark Identification Constants */
    private TalonFX CTRE_motor;
    private TalonFXConfiguration talonFXConfigs;

    /* Motor Constantants */
    private final String loggerBase;
    private final int motorID;
    private final String canBusName;
    private final double gearRatio;
    private final double wheelRadius;
    private final NeutralModeValue idleMode;
    private final boolean inverted;
    private final CTRE_MotionProfile motionProfile;
    private final GravityTypeValue GravityType;

/*Constuctor to inalize values */
public Talon(
    String loggerBase,//Bring in the name of the motor so error can speciffy
    int motorID,
    String canBusName,
    double gearRatio,
    double wheelRadius,
    boolean idle_Brake, 
    boolean inverted,
    Motor_Type motorType,
    CTRE_MotionProfile motionProfile//Holds motor PID, FF, and motion Constrants
  ) {

    //Pass name of motor to the MotorBase class
    super(loggerBase);

  /*Set value to import or set values ot defualt if null */
    this.loggerBase = loggerBase;
    this.motorID = motorID;
    this.canBusName = canBusName;
    this.gearRatio = gearRatio;
    this.wheelRadius = wheelRadius;
    this.motorType = (motorType != null) ? motorType : new MotorTypes.CTRE_Defualt();
    this.motionProfile = (motionProfile != null) ? motionProfile : MotionProfileHelpers.getDisabledCTREProfile();
    this.idleMode = idle_Brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    this.inverted = inverted;
    GravityType = this.motionProfile.gravityType;
    
    configureMotor();
  }

    public void configureMotor(){
        // Motor initialization
        CTRE_motor = new TalonFX(motorID,canBusName);
        talonFXConfigs = new TalonFXConfiguration();

        // Set motor to inverted
        talonFXConfigs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        //Set the ideal mode of the motor
        CTRE_motor.setNeutralMode(idleMode);  

        // Apply current limits (CTRE)
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = motorType.getDefaultAmpLimits()[0];
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = motorType.getDefaultAmpLimits()[1];
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = motorType.getDefaultAmpLimits()[2];
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = motorType.getDefaultAmpLimits()[3];
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        //Changes out put to acount for gearRatio
        talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio;

        // Slot 0 configurations for PID and FF Position Control
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = motionProfile.ff_kS;
        slot0Configs.kV = motionProfile.ff_kV;
        slot0Configs.kA = motionProfile.ff_kA;
        slot0Configs.kP = motionProfile.kP_Pos;
        slot0Configs.kI = motionProfile.kI_Pos;
        slot0Configs.kD = motionProfile.kD_Pos;
        if ( GravityType != null) {
            slot0Configs.GravityType = GravityType; // Gravity compensation: Elevator_Static or Arm_Cosine
        }

        // Slot 1 configurations for PID and FF Velocity Control
        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = motionProfile.velFF_kS;
        slot1Configs.kV = motionProfile.velFF_kV;
        slot1Configs.kA = motionProfile.velFF_kA;
        slot1Configs.kP = motionProfile.kP_Vel;
        slot1Configs.kI = motionProfile.kI_Vel;
        slot1Configs.kD = motionProfile.kD_Vel;

        // Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = motionProfile.cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = motionProfile.acceleration;
        motionMagicConfigs.MotionMagicJerk = motionProfile.jerk;
        motionMagicConfigs.MotionMagicExpo_kV = motionProfile.motionKV;
        motionMagicConfigs.MotionMagicExpo_kA = motionProfile.motionKA;


        // Apply all configurations
        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    }
      
    // ========== Getters ==========

    @Override
    public Motor_Type getMotorType(){
        return motorType;
    }

    @Override
    public Rotation2d getMotorPosition() {
        return Rotation2d.fromRotations(CTRE_motor.getPosition().getValueAsDouble()*gearRatio);
    }

    @Override
    public Rotation2d getMotorVelocity() {
        return Rotation2d.fromRotations(CTRE_motor.getVelocity().getValueAsDouble()*gearRatio*60);
    }

    @Override
    public Rotation2d getMechanismPosition() {
        return Rotation2d.fromRotations(CTRE_motor.getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getMechanismVelocity() {
        return Rotation2d.fromRotations(CTRE_motor.getVelocity().getValueAsDouble()*60);
    }

    @Override
    public double getMechanismTipSpeed() {
        return getMechanismVelocity().getRadians()*wheelRadius/60;
    }

    @Override
    public double getMotorVoltage() {
        return CTRE_motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getMotorAmps() {
        return  CTRE_motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getMotorTemperatureCelsius() {
        return CTRE_motor.getDeviceTemp().getValueAsDouble();
    }

    /**
     * Gets the current internal setpoint (target value) being tracked by the motor controller's 
     * closed-loop control (e.g., Motion Magic). This represents the motor's current intermediate 
     * goal based on its trapezoidal motion profile, not the final target that was originally set.
     *
     * <p>This value is updated by calling {@code refresh()} on the StatusSignal, which forces a 
     * CAN bus read to ensure the latest value is retrieved.</p>
     *
     * @return The current internal setpoint value in the motor’s configured units (e.g., rotations).
     */
    public double getSetPoint() {
        var reference = CTRE_motor.getClosedLoopReference();
        reference.refresh(); // Forces an immediate update from the motor controller
        return reference.getValue();
    }


    // ========== Setters ==========

    @Override
    public void setCurrentMechanismPosition(double rotations) {
        CTRE_motor.setPosition(rotations);
        Logger.recordOutput(loggerBase + "currentMechPosition", rotations);
    }
    
    @Override
    public void setCurrentMotorPosition(double rotations) {
        CTRE_motor.setPosition(rotations * gearRatio);
        Logger.recordOutput(loggerBase + "currentMotorPosition", rotations * gearRatio);
    }
    
    @Override
    public void setTargetMotorPosition(double rotations) {
        MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);
        voltageRequest.Slot = 0;
        CTRE_motor.setControl(voltageRequest.withPosition(rotations / gearRatio));
        Logger.recordOutput(loggerBase + "targetMotorPosition", rotations / gearRatio);
    }
    
    @Override
    public void setTargetMotorVelocity(double rpm) {
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(rpm / gearRatio / 60));
        Logger.recordOutput(loggerBase + "targetMotorVelocity", rpm / gearRatio / 60);
    }
    
    @Override
    public void setTargetMotorVoltage(double volts) {
        CTRE_motor.setVoltage(volts);
        Logger.recordOutput(loggerBase + "targetMotorVoltage", volts);
    }
    
    @Override
    public void setMotorPercent(double percent) {
        CTRE_motor.set(percent);
        Logger.recordOutput(loggerBase + "motorPercent", percent);
    }
    
    @Override
    public void setMotorStop() {
        setMotorPercent(0);
    }
    
    @Override
    public void setTargetMechanismPosition(double rotations) {
        MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);
        voltageRequest.Slot = 0;
        CTRE_motor.setControl(voltageRequest.withPosition(rotations));
        Logger.recordOutput(loggerBase + "targetMechanismPosition", rotations);
    }
    
    @Override
    public void setTargetMechanismVelocity(double rpm) {
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(rpm));
        Logger.recordOutput(loggerBase + "targetMechanismVelocity", rpm);
    }
    
    @Override
    public void setTargetMechanismTipSpeed(double metersPerSecond) {
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(metersPerSecond / (2 * Math.PI * wheelRadius)));
        Logger.recordOutput(loggerBase + "targetMechanismTipSpeed", metersPerSecond / (2 * Math.PI * wheelRadius));
    }
    

    @Override
    public void setIdealState(boolean isBrake) {
        NeutralModeValue State;
        if(isBrake==true){State=NeutralModeValue.Brake;}
        else{State=NeutralModeValue.Coast;}
        CTRE_motor.setNeutralMode(State);
    }

    //====== NonAbstract ===== 
    // These methods are to differnt between encoder to have generaic input

        public void setTunerConstants(int slotNumber,double kS,double kV,double kA,double kP,double kI,double kD) {
        if (slotNumber == 0) { // Position slot
            Slot0Configs slot0 = talonFXConfigs.Slot0;
            slot0.kS = kS;
            slot0.kV = kV;
            slot0.kA = kA;
            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;
    
            if (GravityType != null) {
                slot0.GravityType = GravityType;
            }
    
        } else if (slotNumber == 1) { // Velocity slot
            Slot1Configs slot1 = talonFXConfigs.Slot1;
            slot1.kS = kS;
            slot1.kV = kV;
            slot1.kA = kA;
            slot1.kP = kP;
            slot1.kI = kI;
            slot1.kD = kD;
    
        } else {
            System.err.println("ERROR: Invalid PID slot (" + slotNumber + ") for motor: " + loggerBase);
            return;
        }
    
        // Apply the configuration changes to the motor
        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    }

    public void setMotionConstraints(double cruiseVelocity, double acceleration, double jerk, double expo_kV, double expo_kA) {
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
    
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;
        motionMagicConfigs.MotionMagicExpo_kV = expo_kV;
        motionMagicConfigs.MotionMagicExpo_kA = expo_kA;
    
        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    }
    

    public void setAmpLimits(int SupplyCurrentLimit, double SupplyCurrentLowerTime, int SupplyCurrentLowerLimit, int StatorCurrentLimit) {
        var ampLimitConfigs = talonFXConfigs.CurrentLimits;
        ampLimitConfigs.SupplyCurrentLimit = SupplyCurrentLimit;
        ampLimitConfigs.SupplyCurrentLowerTime = SupplyCurrentLowerTime;
        ampLimitConfigs.SupplyCurrentLowerLimit = SupplyCurrentLowerLimit;
        ampLimitConfigs.SupplyCurrentLimitEnable = true;

        ampLimitConfigs.StatorCurrentLimit = StatorCurrentLimit;
        ampLimitConfigs.StatorCurrentLimitEnable = true;

        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    }

    public CurrentLimitsConfigs getCurrentLimits() {
        return talonFXConfigs.CurrentLimits;
    }

}