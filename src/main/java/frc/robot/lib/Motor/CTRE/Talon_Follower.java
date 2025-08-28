package frc.robot.lib.Motor.CTRE;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.Motor.FollowerMotorBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Talon_Follower extends FollowerMotorBase {

    /*Motor and Spark Identification Constants */
    private TalonFX CTRE_motor;
    private TalonFXConfiguration talonFXConfigs;

    /* Motor Constantants */
    private final int motorID;
    private final int leaderID;
    private final String canBusName;
    private final double gearRatio;
    private final double wheelRadius;
    private final NeutralModeValue idleMode;
    private final boolean invertedFromLeader;
    private final CurrentLimitsConfigs LeaderCurrentLimitConfig;

/*Constuctor to initialize values */
public Talon_Follower(
    String loggerBase,//Bring in the name of the motor so error can speciffy
    int motorID,
    int leaderID,
    String canBusName,
    double gearRatio,
    double wheelRadius,
    boolean idle_Brake, 
    boolean invertedFromLeader,
    CurrentLimitsConfigs LeaderCurrentLimitConfig
  ) {
    //Pass name of motor to the MotorBase class
    super(loggerBase);

  /*Set value to import or set values ot defualt if null */
    this.motorID = motorID;
    this.leaderID=leaderID;
    this.canBusName = canBusName;
    this.gearRatio = gearRatio;
    this.wheelRadius = wheelRadius;
    this.idleMode = idle_Brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    this.invertedFromLeader=invertedFromLeader;
    this.LeaderCurrentLimitConfig=LeaderCurrentLimitConfig;

    configureMotor();
  }

    public void configureMotor(){
        // Motor initialization
        CTRE_motor = new TalonFX(motorID,canBusName);
        talonFXConfigs = new TalonFXConfiguration();

        // Set motor to inverted
        CTRE_motor.setNeutralMode(idleMode);  

        // Apply current limits (CTRE)
        talonFXConfigs.CurrentLimits = LeaderCurrentLimitConfig;

        //Changes out put to acount for gearRatio
        talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio;

        // Apply all configurations
        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
        CTRE_motor.setControl(new Follower(leaderID, invertedFromLeader));
    }
      
    // ========== Getters ==========
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

    // ========== Setters ==========

    @Override
    public void setIdealState(boolean isBrake) {
        NeutralModeValue State;
        if(isBrake==true){State=NeutralModeValue.Brake;}
        else{State=NeutralModeValue.Coast;}
        CTRE_motor.setNeutralMode(State);
    }

    //====== NonAbstract ===== 
    // These methods are to differnt between encoder to have generaic input

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

}
