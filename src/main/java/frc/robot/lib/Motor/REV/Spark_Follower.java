package frc.robot.lib.Motor.REV;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.Motor.FollowerMotorBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class Spark_Follower extends FollowerMotorBase{

    /*Motor and Spark Identification Constants */
        private SparkBase motor;
        private SparkBaseConfig motor_config;
        private RelativeEncoder motor_encoder;
        
        private final int motorID;
        private final int leaderID;
        private final SparkBaseConfig leader_config;
        private final double gearRatio;
        private final double wheelRadius;
        private final boolean invertedFromLeader;

    
    
    /*Constuctor to initialize values */
    public Spark_Follower(
        String loggerBase,//Bring in the name of the motor so error can speciffy
        int motorID,
        int leaderID,
        SparkBaseConfig leader_config,
        double gearRatio,
        double wheelRadius,
        boolean invertedFromLeader,
        boolean isSparkFlex
      ) {
        //Pass name of motor to the MotorBase class
        super(loggerBase);
    
      /*Set value to import or set values ot defualt if null */
        this.motorID = motorID;
        this.leaderID = leaderID;
        this.leader_config = leader_config;
        this.gearRatio = gearRatio;
        this.wheelRadius = wheelRadius;
        this.invertedFromLeader=invertedFromLeader;
    
        configureMotor(isSparkFlex);
      }
    
      public void configureMotor(boolean isSparkFlex) {

        if (isSparkFlex) {
            motor = new SparkFlex(motorID, MotorType.kBrushless);
            motor_config = new SparkFlexConfig();
        } else {
            motor = new SparkMax(motorID, MotorType.kBrushless);
            motor_config = new SparkMaxConfig();
        }

        motor_encoder = motor.getEncoder(); 

            // Follower setup if motor should follow the leader
            motor_config
            .apply(leader_config)
            .follow(leaderID, invertedFromLeader);

    // Apply the configuration to the motor with reset and persist modes
    motor.configure(motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}


    // ========== Getters ==========
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

    public void setAmpLimits(int stallLimit, int freeLimit, int limitRpm) {
        SparkBaseConfig config = newEmptyConfig();
    
        config.smartCurrentLimit(stallLimit, freeLimit, limitRpm);
    
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    

}
