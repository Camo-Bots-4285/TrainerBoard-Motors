package frc.robot.util.CTRE_Hardware;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import frc.robot.Robot;

public class CTRE_Talon extends SubsystemBase {

    // Define class variables
    private TalonFX CTRE_motor;
    private TalonFXConfiguration talonFXConfigs;


  /* Motor Constantants */
    public String MotorIdentification;
    public double GearRatio;
    public double WheelRadius;
    public NeutralModeValue RunningIdle;
    public int StatorCurrentLimit = 100;
    public int SupplyCurrentLimit = 70;
    public double SupplyCurrentLowerTime = 1.0;
    public int SupplyCurrentLowerLimit = 40;
    public double OverHeatTemp= 75;

  /*Emergency Stop Vaibales */
    // Predefined values (can be changed later as needed)
    public int numLoops = 5;  // Number of loops to store readings
    private  int requiredLoopsToShutOff = 3;  // The number of loops that must exceed the limit to shut off the motor
    public int EmergencyStopCurrent = 50;

    private double[] currentReadings = new double[numLoops];  // Array to store the current readings
    private int currentIndex = 0;  // Keeps track of the current index in the readings array
    private int excessCount = 0;  // Tracks how many times the current exceeded the limit

    public double emergenyStopTimeDuration = 3;
    public double emergenyStopTime= - emergenyStopTimeDuration;


  /*Motion Constants */
    public double[] TunnerValues;
    public double [] MotionConstants;
    public GravityTypeValue GravityType;
        
    public CTRE_Talon(
        String MotorIdentification,
        String CANLoop,
        int[] Motor_int,//Bring in int motor variables
        double[] Motor_double, //Bring in double values
        boolean[] Motor_boolean, //Bring in motor states
        double[] TunnerValues,//Holds motor FF and PID for position and velocity
        double[] MotionConstants//Holds motor motion values
    ) {

    /*Set value to import or set values ot defualt if null */
      this.MotorIdentification = MotorIdentification;

      StatorCurrentLimit = Motor_int[1];
      SupplyCurrentLimit = Motor_int[2];
      SupplyCurrentLowerLimit = Motor_int[3];
      EmergencyStopCurrent = Motor_int[4];

      GearRatio = Motor_double[0];
      WheelRadius = Motor_double[1];
      OverHeatTemp=Motor_double[3];
      SupplyCurrentLowerTime=Motor_double[4];

      if(Motor_boolean[0]==true){RunningIdle = NeutralModeValue.Brake;}else{RunningIdle = NeutralModeValue.Coast;}
      if(Motor_boolean[1]==true){talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;}else{talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;}

      if(TunnerValues[6]==0){GravityType=null;}else if(TunnerValues[6]==1){GravityType=GravityTypeValue.Elevator_Static;}else if(TunnerValues[6]==2){GravityType=GravityTypeValue.Arm_Cosine;}

      this.TunnerValues=TunnerValues;
      this.MotionConstants=MotionConstants;

        // Motor initialization
        CTRE_motor = new TalonFX(Motor_int[0],CANLoop);
        talonFXConfigs = new TalonFXConfiguration();

        
        // Set motor to inverted
        CTRE_motor.setNeutralMode(RunningIdle);  



        // Apply current limits (CTRE)
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = SupplyCurrentLowerTime;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = SupplyCurrentLowerLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        //Changes out put to acount for gearRatio
        talonFXConfigs.Feedback.SensorToMechanismRatio = GearRatio;


        // Slot 0 configurations for PID and FF Position Control
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = TunnerValues[1];//0.25;
        slot0Configs.kV = TunnerValues[2];//0.12;
        slot0Configs.kA = TunnerValues[3];//0.01;
        slot0Configs.kP = TunnerValues[4];//6;
        slot0Configs.kI = TunnerValues[5];//0;
        slot0Configs.kD = TunnerValues[6];//0.1;
        if(GravityType!=null){slot0Configs.GravityType = GravityType;} //There is FF for arm and elavator to over come gravity




        // Slot 0 configurations for PID and FF Velocity Control
        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = TunnerValues[8];
        slot1Configs.kV = TunnerValues[9];
        slot1Configs.kA = TunnerValues[10];
        slot1Configs.kP = TunnerValues[11];
        slot1Configs.kI = TunnerValues[12];
        slot1Configs.kD = TunnerValues[13];
        


        // Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MotionConstants[0];
        motionMagicConfigs.MotionMagicAcceleration = MotionConstants[1];
        motionMagicConfigs.MotionMagicJerk = MotionConstants[2];
        motionMagicConfigs.MotionMagicExpo_kV = MotionConstants[3];
        motionMagicConfigs.MotionMagicExpo_kA = MotionConstants[4];


        // Apply all configurations
        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    };

    /* Getter - Get values from the motors */
    public int getMotorID(){
        return CTRE_motor.getDeviceID();
    }

    public double[] get_Follower_Motor_doubles(){
        double[] Follower_Motor_double = {GearRatio,WheelRadius,OverHeatTemp};
        return Follower_Motor_double;
      }

    public Rotation2d getMotorPosition() {//Get the position of the motor and translates it to Rotation2D
        return Rotation2d.fromRotations(CTRE_motor.getPosition().getValueAsDouble()/GearRatio);  // Return position of the motor
    }

    public Rotation2d getMotorVelocity_RPM() {//Get the velocity of the motor and translates it to Rotation2D per minute
        return Rotation2d.fromRotations(CTRE_motor.getVelocity().getValueAsDouble()/GearRatio*60);  // Return velocity of the motor
    }

    public double getVoltage() {//Get the voltage(volts) given to the motor controller
        return CTRE_motor.getMotorVoltage().getValueAsDouble();// Return voltage (volts) of the motor
    }

    public double getAmps() {//Get the current(amps) given to the motor
        return CTRE_motor.getStatorCurrent().getValueAsDouble();  // Return current (amps) of the motor
    }

    public double getTemputerInCelcius(){//Get the temputarue of the motor
        return CTRE_motor.getDeviceTemp().getValueAsDouble();
      }

    public Rotation2d getMechanismPosition() {//Get the position after gear ratio and translates it to Rotation2D
        return Rotation2d.fromRotations(CTRE_motor.getPosition().getValueAsDouble());
    }

    public Rotation2d getMechanismVelocity_RPM() {//Get the velocity after gear ratio and translates it to Rotation2D per minute
        return Rotation2d.fromRotations(CTRE_motor.getVelocity().getValueAsDouble()*60);
    }

    public double getTipSpeed() {//Get the velocity after gear ratio and wheel circomfernce m/s
        return getMechanismVelocity_RPM().getRadians()*WheelRadius/60; 
    }

    public boolean needEmergenyStop(){ //Check the amp limit and sees if motor need to Stop
        // Add the current reading to the array, replacing the old one at the current index
        currentReadings[currentIndex] = getAmps();
  
        // If the current reading exceeds the emergency limit, increment excessCount
        if (getAmps() > EmergencyStopCurrent) {
            excessCount++;
        }
  
        // Update the current index, wrapping around to 0 if it exceeds numLoops
        currentIndex = (currentIndex + 1) % numLoops;
  
        // Check if enough loops have exceeded the limit
        if (excessCount >= requiredLoopsToShutOff) {
            return true;  // The motor should be shut off
        }
  
        return false;  // The motor should not be shut off yet
      }

      public boolean motorOverHeating(){//Returns true if temputere is above value
        if (getTemputerInCelcius()>= OverHeatTemp){//Checks tempurature
          return true;
        }
        return false;
      }


    /* Setter - Used to set the values of motors */
    public void setCurrentMechanismPosition(double position_rotations) { // Set motor encoder position
        CTRE_motor.setPosition(position_rotations); 
    }

    public void setTargetMotorPosition(double setPoint_rotations) {//Sets the target position for the motor
        MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);
        voltageRequest.Slot = 0;
        CTRE_motor.setControl(voltageRequest.withPosition(setPoint_rotations*GearRatio));  // Set motor target position using motion magic

    }

    public void setTargetMotorVelocity_RPM(double setPoint_rotations_per_minute) {//Sets the target velocity for the motor
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(setPoint_rotations_per_minute*GearRatio/60));  // Set motor target velocity
    }

    public void setTargetMotorVoltage(double setPoint_volts){//Sets the target voltage for the motor
      CTRE_motor.setVoltage(setPoint_volts);
    }


    //TODO add torgue contorll?

    public void setMotorPercent(double percent) {//Sets the target percent power for the motor
        // Set motor speed by percentage of power (0 to 1)
        CTRE_motor.set(percent); 
    }

    public void setMotorStop() {//Sets the percent power for the motor to zero
        setMotorPercent(0);  // Stop motor by setting target velocity to 0
    }

    public void setTargetMechanismPosition(double setPoint_rotations) {
        MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);
        voltageRequest.Slot = 0;
        CTRE_motor.setControl(voltageRequest.withPosition(setPoint_rotations));  // Set motor target position using motion magic
    }

    public void setTargteMechanismVelocity(double setPoint_rotations_per_minute) {
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(setPoint_rotations_per_minute)); // Set motor target velocity
    
    }

    public void setTargetMechanismTipSpeed(double meter_per_second) {
        VelocityVoltage voltageRequest = new VelocityVoltage(0);
        voltageRequest.Slot = 1;
        CTRE_motor.setControl(voltageRequest.withVelocity(meter_per_second/(2*Math.PI*WheelRadius))); // Set motor target velocity// Set the mechanism tip speed (not implemented in the original code)
    }

    public void setIdealState(Boolean is_Break) {
        NeutralModeValue State;
        if(is_Break==true){State=NeutralModeValue.Brake;}
        else{State=NeutralModeValue.Coast;}
        CTRE_motor.setNeutralMode(State);
    }

    public void setTunerConstants(double[] new_TunnerValues, int SlotNumber) {
      //Please make sure the array is correct copy from robot containor
      //if no change is need put (Double) null

      //Slot options
      //0-Positon
      //1-Velocity
      //2-Emty
      //3-Empty
      double[] set_TunnerValues=TunnerValues;

        if(SlotNumber==0){//Slot choosen is zero
        for (int i = 0; i <= 6; i++) {//Runs for loop to check is values have change
        if (new_TunnerValues[i] != (Double) null){set_TunnerValues[i]=new_TunnerValues[i];}//If values have changed changes them
          }
          var slot0Configs = talonFXConfigs.Slot0;
          slot0Configs.kS = TunnerValues[1];//0.25;
          slot0Configs.kV = TunnerValues[2];//0.12;
          slot0Configs.kA = TunnerValues[3];//0.01;
          slot0Configs.kP = TunnerValues[4];//6;
          slot0Configs.kI = TunnerValues[5];//0;
          slot0Configs.kD = TunnerValues[6];//0.1;
          if(GravityType!=null){slot0Configs.GravityType = GravityType;}} //There is FF for arm and elavator to over come gravity// Enable/Disable PID for position control}

      else if(SlotNumber==1){//Slot choosen is one
        for (int i = 8; i <= 13; i++) {//Runs for loop to check is values have change
          if (new_TunnerValues[i] != (Double) null){set_TunnerValues[i]=new_TunnerValues[i];}//If values have changed changes them
            }   
            var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kS = TunnerValues[8];//0.25;
            slot0Configs.kV = TunnerValues[9];//0.12;
            slot0Configs.kA = TunnerValues[10];//0.01;
            slot0Configs.kP = TunnerValues[11];//6;
            slot0Configs.kI = TunnerValues[12];//0;
            slot0Configs.kD = TunnerValues[13];//0.1;
            } //There is FF for arm and elavator to over come gravity// Enable/Disable PID for position control}

      //If slot selection fails then gives error 
      else{System.err.println("ERROR:"+MotorIdentification+"failed to set tunner constants");}

      CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);  
    }

    public void setMotionConstaints(double[] NewMotionConstants) {
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        double[] set_MotionConstants = MotionConstants;
        for (int i = 1; i <= 5; i++) {//Runs for loop to check is values have change
        if(NewMotionConstants[i]!=(Double)null){set_MotionConstants[i]= NewMotionConstants[i];}}

        motionMagicConfigs.MotionMagicCruiseVelocity = set_MotionConstants[0];
        motionMagicConfigs.MotionMagicAcceleration = set_MotionConstants[1];
        motionMagicConfigs.MotionMagicJerk = set_MotionConstants[2];
        motionMagicConfigs.MotionMagicExpo_kV = set_MotionConstants[3];
        motionMagicConfigs.MotionMagicExpo_kA = set_MotionConstants[4];

        CTRE_motor.getConfigurator().apply(talonFXConfigs, 0.010);
    }

    public void runMotorProdection() {
        /* Check if need to be in emergency stop and print error while in that state */
        if (needEmergenyStop()) {
            emergenyStopTime = Robot.Time;
            setMotorStop();
            System.err.println("WARNING" + MotorIdentification + ": Emergency Stopped");
        }
        /* Check if the motor has been emergency stopped within the emergencyStopTimeDuration */
        else if (Robot.Time - emergenyStopTimeDuration < emergenyStopTime) {
            setMotorStop();
            System.err.println("WARNING" + MotorIdentification + ": Emergency Stopped Cool Down");
        }
    }
    
    public void runTemperatureWarning() {//Displays warning if the motor is too hot.
        if(motorOverHeating()){
            System.err.println("WARNING:"+MotorIdentification+":Motor is over heating");
        }
      
    }
}
