package frc.robot.util.Rev_Hardware;


import frc.robot.Robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;



public class REV_Flex extends SubsystemBase {

  /*Motor and Spark Identification Constants */
    SparkFlex REV_motor;
    SparkClosedLoopController Rev_Flex_controller;
    SparkFlexConfig Rev_Flex_config;
    RelativeEncoder Rev_Flex_encoder;
  /* Motor Constantants */
    public String MotorIdentification;
    public double GearRatio;
    public double WheelRadius;
    public IdleMode RunningIdle;
    public int[] Motor_int_values;
    public double OverHeatTemp;

  /*Emergency Stop Vaibales */
    // Predefined values (can be changed later as needed)
    public int numLoops = 5;  // Number of loops to store readings
    private  int requiredLoopsToShutOff = 3;  // The number of loops that must exceed the limit to shut off the motor

    private double[] currentReadings = new double[numLoops];  // Array to store the current readings
    private int currentIndex = 0;  // Keeps track of the current index in the readings array
    private int excessCount = 0;  // Tracks how many times the current exceeded the limit

    public double emergenyStopTimeDuration = 3;
    public double emergenyStopTime= - emergenyStopTimeDuration;

  /*Motion Constants */
    public double velocity;
    public double accleration;
    public double allowedError;
    public double[] TunnerValues;

  /*Constuctor to inalize values */
  public REV_Flex(
      String MotorIdentification,//Bring in the name of the motor so error can speciffy
      int[] Motor_int_values,//Bring in int motor variables
      double[] Motor_double_values, //Bring in double values
      boolean[] Motor_boolean_values, //Bring in motor states
      double[] TunnerValues,//Holds motor FF and PID for position and velocity
      double[] MotionConstants//Holds motor motion values
    ) {

    /*Set value to import or set values ot defualt if null */
      this.MotorIdentification = MotorIdentification;
      GearRatio = Motor_double_values[0];
      WheelRadius = Motor_double_values[1];
      OverHeatTemp=Motor_double_values[2];

      velocity = MotionConstants[0];
      accleration = MotionConstants[1];
      allowedError = MotionConstants[2];
      this.TunnerValues = TunnerValues;
    
     this.Motor_int_values = Motor_int_values;
      if (Motor_boolean_values[0] ==true){RunningIdle=IdleMode.kBrake;}else{RunningIdle=IdleMode.kCoast;}

    /*Identifies Motor and encoder*/
        REV_motor = new SparkFlex(Motor_int_values[0], MotorType.kBrushless);
        Rev_Flex_controller = REV_motor.getClosedLoopController();
        Rev_Flex_config = new SparkFlexConfig();
        Rev_Flex_encoder = REV_motor.getEncoder();
        
        /*Sets config for  motor */
        Rev_Flex_config
        .smartCurrentLimit(Motor_int_values[1],Motor_int_values[2],Motor_int_values[3])
        .inverted(Motor_boolean_values[1])
        .idleMode(RunningIdle);
        
          /*Configures motion control variables */
          Rev_Flex_config.closedLoop
          // Set gains for position control in slot 0
          .pidf(TunnerValues[0],TunnerValues[1],TunnerValues[2],TunnerValues[3],ClosedLoopSlot.kSlot0)
          .iZone(TunnerValues[4],ClosedLoopSlot.kSlot0)
          .iMaxAccum(TunnerValues[5],ClosedLoopSlot.kSlot0)
          .outputRange(TunnerValues[6],TunnerValues[7],ClosedLoopSlot.kSlot0)
                    
                    
          // Set gains for velocity control in slot 1
          .pidf(TunnerValues[8],TunnerValues[9],TunnerValues[10],TunnerValues[11],ClosedLoopSlot.kSlot1)
          .iZone(TunnerValues[12],ClosedLoopSlot.kSlot1)
          .iMaxAccum(TunnerValues[13],ClosedLoopSlot.kSlot1)
          .outputRange(TunnerValues[14],TunnerValues[15],ClosedLoopSlot.kSlot1)

          // Set gains for velocity control in slot 2
          .pidf(TunnerValues[16],TunnerValues[17],TunnerValues[18],TunnerValues[19],ClosedLoopSlot.kSlot2)
          .iZone(TunnerValues[20],ClosedLoopSlot.kSlot2)
          .iMaxAccum(TunnerValues[21],ClosedLoopSlot.kSlot2)
          .outputRange(TunnerValues[22],TunnerValues[23],ClosedLoopSlot.kSlot2);

          // Slot 3 is avaliable for more control
                    
        /*Configures motion limits*/
          Rev_Flex_config.closedLoop.maxMotion
          .maxVelocity(MotionConstants[0])
          .maxAcceleration(MotionConstants[1])
          .allowedClosedLoopError(MotionConstants[2]);
        
          /*Set update frequency */
          Rev_Flex_config.signals.primaryEncoderPositionPeriodMs(10);
                 
          /*Applies configuration */
          REV_motor.configure(Rev_Flex_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  

  /*Getter-Get values from the motors */
    public SparkFlexConfig getConfig(){//Get the config to pass thought to a follower motor
      return Rev_Flex_config;
    }

    public SparkFlex getMotor(){//Get the motor to pass thought to a follower motor
      return REV_motor;
    }

    public int get_EmergencyStopAmps(){
      return Motor_int_values[4];
    }

    public double[] get_Follower_Motor_doubles(){
      double[] Follower_Motor_double_values = {GearRatio,WheelRadius,OverHeatTemp};
      return Follower_Motor_double_values;
    }

    public Rotation2d getMotorPosition(){//Get the position of the motor and translates it to Rotation2D
      return Rotation2d.fromRotations(Rev_Flex_encoder.getPosition());
    }

    public Rotation2d getMotorVelocity(){//Get the velocity of the motor and translates it to Rotation2D per minute
      return Rotation2d.fromRotations(Rev_Flex_encoder.getVelocity());
    }

    public double getVoltage(){//Get the voltage(volts) given to the motor controller
      return REV_motor.getBusVoltage();
    }

    public double getAmps(){//Get the current(amps) given to the motor
      return REV_motor.getOutputCurrent();
    }

    public double getTemputerInCelcius(){//Get the temputarue of the motor
      return REV_motor.getMotorTemperature();
    }

    public  Rotation2d getMechanismPosition(){//Get the position after gear ratio and translates it to Rotation2D
      return  Rotation2d.fromRotations(Rev_Flex_encoder.getPosition()*GearRatio);
    }

    public Rotation2d getMechanismVelocity(){//Get the velocity after gear ratio and translates it to Rotation2D per minute
      return  Rotation2d.fromRotations(Rev_Flex_encoder.getVelocity()*GearRatio);
    }

    public double getTipSpeed(){//Get the velocity after gear ratio and wheel circomfernce m/s
      return getMechanismVelocity().getRadians()*WheelRadius*GearRatio;
    }

    public boolean needEmergenyStop(){ //Check the amp limit and sees if motor need to Stop
      // Add the current reading to the array, replacing the old one at the current index
      currentReadings[currentIndex] = getAmps();

      // If the current reading exceeds the emergency limit, increment excessCount
      if (getAmps() > Motor_int_values[5]) {
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


  /*Setter-Used to set the values of motors*/
    public void setCurrentMechanismPosition(double position_rotations){//Set current positon of the encoder
      Rev_Flex_encoder.setPosition(position_rotations);
    }

    public void setTargetMotorPosition(double setPoint_rotations){//Sets the target position for the motor
      Rev_Flex_controller.setReference(setPoint_rotations*GearRatio, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setTargetMotorVelocity(double setPoint_rotations_per_minute){//Sets the target velocity for the motor
      Rev_Flex_controller.setReference(setPoint_rotations_per_minute*GearRatio, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void setTargetMotorVoltage(double setPoint_volts){//Sets the target voltage for the motor
      Rev_Flex_controller.setReference(setPoint_volts, ControlType.kVoltage, ClosedLoopSlot.kSlot2);
    }

    public void setMotorPercent(double percent){//Sets the target percent power for the motor
      REV_motor.set(percent);
    }

    public void setMotorStop(){//Sets the percent power for the motor to zero
      REV_motor.set(0);
    }

    public void setTargetMechanismPosition(double setPoint_rotations){//Sets the target positon for the motor after the grear ratio
      Rev_Flex_controller.setReference(setPoint_rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setTargetMechanismVelocity(double setPoint_rotations_per_minute){//Sets the target velocity for the motor after the grear ratio
      Rev_Flex_controller.setReference(setPoint_rotations_per_minute, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void setTargetMechanismTipSpeed(double meter_per_second){//Sets the target velocity for the motor after the grear ratio and wheel circumfernce
      setTargetMechanismVelocity(meter_per_second/(2*Math.PI*WheelRadius));
    }


    public void setIdealState(boolean is_Break){//Sets the motor to an ideal state should be called once
      IdleMode SetIdleState;
      if(is_Break == true){SetIdleState=IdleMode.kBrake;}
      else{SetIdleState=IdleMode.kCoast;}

      //Make new config
      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(SetIdleState);
                
      //Configures motor will go back to defalt after power cycle 
      REV_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void setTunerConstants(double new_TunnerValues[], int SlotNumber){//Sets the motor tunerConstants should be called once 
      //Please make sure the array is correct copy from robot containor
      //if no change is need put (Double) null

      //Slot options
      //0-Positon
      //1-Velocity
      //2-Voltage
      //3-Empty

      //Make new config
      SparkMaxConfig config = new SparkMaxConfig();
      double set_TunnerValues[]=TunnerValues;

      //Check which slot the configure should be applies to
      if(SlotNumber==0){//Slot choosen is zero
        for (int i = 0; i <= 7; i++) {//Runs for loop to check is values have change
        if (new_TunnerValues[i] != (Double) null){set_TunnerValues[i]=new_TunnerValues[i];}//If values have changed changes them
          }
          config.closedLoop//Define new config values
          .pidf(TunnerValues[0],TunnerValues[1],TunnerValues[2],TunnerValues[3],ClosedLoopSlot.kSlot0)
          .iZone(TunnerValues[4],ClosedLoopSlot.kSlot0)
          .iMaxAccum(TunnerValues[5],ClosedLoopSlot.kSlot0)
          .outputRange(TunnerValues[6],TunnerValues[7],ClosedLoopSlot.kSlot0);}

      else if(SlotNumber==1){//Slot choosen is one
        for (int i = 8; i <= 15; i++) {//Runs for loop to check is values have change
          if (new_TunnerValues[i] != (Double) null){set_TunnerValues[i]=new_TunnerValues[i];}//If values have changed changes them
            }   
      config.closedLoop //Define new config values
      .pidf(TunnerValues[8],TunnerValues[9],TunnerValues[10],TunnerValues[11],ClosedLoopSlot.kSlot1)
      .iZone(TunnerValues[12],ClosedLoopSlot.kSlot1)
      .iMaxAccum(TunnerValues[13],ClosedLoopSlot.kSlot1)
      .outputRange(TunnerValues[14],TunnerValues[15],ClosedLoopSlot.kSlot1);}

      else if(SlotNumber==2){//Slot choosen is two
        for (int i = 16; i <= 23; i++) {//Runs for loop to check is values have change
          if (new_TunnerValues[i] != (Double) null){set_TunnerValues[i]=new_TunnerValues[i];}//If values have changed changes them
            }
            config.closedLoop//Define new config values 
            .pidf(TunnerValues[16],TunnerValues[17],TunnerValues[18],TunnerValues[19],ClosedLoopSlot.kSlot2)
            .iZone(TunnerValues[20],ClosedLoopSlot.kSlot2)
            .iMaxAccum(TunnerValues[21],ClosedLoopSlot.kSlot2)
            .outputRange(TunnerValues[22],TunnerValues[23],ClosedLoopSlot.kSlot2);}
      //If slot selection fails then gives error 
      else{System.err.println("ERROR:"+MotorIdentification+"failed to set tunner constants");}

      //Configures motor will go back to defalt after power cycle 
      REV_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //Set new motion limits if not change is need put (Double) null
    public void setNewMotionConstaints(double new_velocity, double new_accleration, double new_allowedError){
      //Make new config
      SparkMaxConfig config = new SparkMaxConfig();

      //Define method varaibles
      double set_velocity;
      double set_accleration;
      double set_allowedError;

      //If values are null set them to default if not set them to new values
      if(new_velocity!=(Double) null){set_velocity=new_velocity;}else{set_velocity=velocity;}
      if(new_accleration!=(Double) null){set_accleration=new_accleration;}else{set_accleration=accleration;}
      if(new_allowedError!=(Double) null){set_allowedError=new_allowedError;}else{set_allowedError=allowedError;}

      //Configures new motion limits
      config.closedLoop.maxMotion
      .maxVelocity(set_velocity)
      .maxAcceleration(set_accleration)
      .allowedClosedLoopError(set_allowedError);

      //Configures motor will go back to defalt after power cycle 
      REV_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

      // Method to check if the motor should be shut off based on the current reading
    public void runCrashProdection() {
        /*Check if need to be in emergeny stop and print error while is in that state */
        /*Make sure to add end every command using needEmergenyStop() in finshed if using */

        /*Check if the motor needs emergeny stop*/
        if(needEmergenyStop()){
            emergenyStopTime= Robot.Time;
            setMotorStop();
            System.err.println("WARNING"+MotorIdentification+":Emergency Stopped");
        }
        /*Check if the motor has been emergeny stop withing the emergenyStopTimeDuration4*/
        else if(Robot.Time-emergenyStopTimeDuration < emergenyStopTime){
          setMotorStop();
            System.err.println("WARNING"+MotorIdentification+":Emergency Stopped Cool Down");
        }
    }

    public void runTemperatureWarning() {//Displays warning if the motor is too hot.
      if(motorOverHeating()){
          System.err.println("WARNING:"+MotorIdentification+":Motor is over heating");
      }
    
  }

}








