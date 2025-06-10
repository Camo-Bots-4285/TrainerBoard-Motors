package frc.robot.lib.Rev_Hardware;

import frc.robot.Robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;



public class REV_Follower extends SubsystemBase {

  /*Motor and Spark Identification Constants */
    SparkBase Rev_Follower_Motor;
    SparkClosedLoopController Rev_Follower_Controller;
    SparkBaseConfig Rev_Follower_config;
    RelativeEncoder Rev_Follower_encoder;

  /* Motor Constantants */
    public String MotorIdentification;

  /*Emergency Stop Vaibales */
    // Predefined values (can be changed later as needed)
    public int numLoops = 5;  // Number of loops to store readings
    private  int requiredLoopsToShutOff = 3;  // The number of loops that must exceed the limit to shut off the motor

    private double[] currentReadings = new double[numLoops];  // Array to store the current readings
    private int currentIndex = 0;  // Keeps track of the current index in the readings array
    private int excessCount = 0;  // Tracks how many times the current exceeded the limit

    public double emergenyStopTimeDuration = 3;
    public double emergenyStopTime= - emergenyStopTimeDuration;

    public int emergencyStopTimeAmps;
    public double[] Follower_PhysicalCharacteristics; //Bring in double values

  /*Motion Constants */
    public double velocity;
    public double accleration;
    public double allowedError;
    public double[] TunnerValues;

  /*Constuctor to inalize values */
  public REV_Follower(
      String MotorIdentification,//Bring in the name of the motor so error can speciffy
      int MotorID,//Bring in int motor variables
      int emergencyStopTimeAmps,
      double[] Follower_PhysicalCharacteristics, //Bring in double values
      SparkBaseConfig LeaderConfig,//Bring in the leader's Config
      SparkBase LeaderMotor,//Brings in the leader's motor to copy
      boolean invertedFormLeader,//Bring in the inversion settings
      boolean is_Spark_Max

    ) {
      this.Follower_PhysicalCharacteristics=Follower_PhysicalCharacteristics;
      this.emergencyStopTimeAmps=emergencyStopTimeAmps;
    if(is_Spark_Max==true){
      Rev_Follower_Motor = new SparkMax(MotorID, MotorType.kBrushless);
      Rev_Follower_config = new SparkMaxConfig();

      //Here is some follower code if you want to motor to do the same thing
          Rev_Follower_config
          .apply(LeaderConfig)
          .follow(LeaderMotor,invertedFormLeader);
    }
    else{
      Rev_Follower_Motor = new SparkFlex(MotorID, MotorType.kBrushless);
      Rev_Follower_config = new SparkFlexConfig();

      //Here is some follower code if you want to motor to do the same thing
          Rev_Follower_config
          .apply(LeaderConfig)
          .follow(LeaderMotor,invertedFormLeader);
    }

    Rev_Follower_Motor.configure(Rev_Follower_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
//TODOnow in mehcanical unsits swape
  /*Getter-Get values from the motors */

  public Rotation2d getMotorPosition(){//Get the position of the motor and translates it to Rotation2D
    return Rotation2d.fromRotations(Rev_Follower_Motor.getEncoder().getPosition());
  }

  public Rotation2d getMotorVelocity_RPM(){//Get the velocity of the motor and translates it to Rotation2D per minute
    return Rotation2d.fromRotations(Rev_Follower_Motor.getEncoder().getVelocity());
  }

  public double getVoltage(){//Get the voltage(volts) given to the motor controller
    return Rev_Follower_Motor.getBusVoltage();
  }

  public double getAmps(){//Get the current(amps) given to the motor
    return Rev_Follower_Motor.getOutputCurrent();
  }

  public double getTemputerInCelcius(){//Get the temputarue of the motor
    return Rev_Follower_Motor.getMotorTemperature();
  }

  public  Rotation2d getMechanismPosition(){//Get the position after gear ratio and translates it to Rotation2D
    return  Rotation2d.fromRotations(Rev_Follower_Motor.getEncoder().getPosition()*Follower_PhysicalCharacteristics[0]);
  }

  public Rotation2d getMechanismVelocity_RPM(){//Get the velocity after gear ratio and translates it to Rotation2D per minute
    return  Rotation2d.fromRotations(Rev_Follower_Motor.getEncoder().getVelocity()*Follower_PhysicalCharacteristics[0]);
  }

  public double getTipSpeed(){//Get the velocity after gear ratio and wheel circomfernce m/s
    return getMechanismVelocity_RPM().getRadians()*Follower_PhysicalCharacteristics[1]*Follower_PhysicalCharacteristics[0];
  }

  public boolean needEmergenyStop(){ //Check the amp limit and sees if motor need to Stop
    // Add the current reading to the array, replacing the old one at the current index
    currentReadings[currentIndex] = getAmps();

    // If the current reading exceeds the emergency limit, increment excessCount
    if (getAmps() > emergencyStopTimeAmps) {
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
    if (getTemputerInCelcius()>= Follower_PhysicalCharacteristics[2]){//Checks tempurature
      return true;
    }
    return false;
  }

        // Method to check if the motor should be shut off based on the current reading
        public void runCrashProdection() {
          /*Check if need to be in emergeny stop and print error while is in that state */
          /*Make sure to add end every command using needEmergenyStop() in finshed if using */
  
          /*Check if the motor needs emergeny stop*/
          if(needEmergenyStop()){
              emergenyStopTime= Robot.Time;
              System.err.println("WARNING"+MotorIdentification+":Emergency Stopped but is follower");
          }
          /*Check if the motor has been emergeny stop withing the emergenyStopTimeDuration4*/
          else if(Robot.Time-emergenyStopTimeDuration < emergenyStopTime){
              System.err.println("WARNING"+MotorIdentification+":Emergency Stopped Cool Down but is follower");
          }
      }
  
      public void runTemperatureWarning() {//Displays warning if the motor is too hot.
        if(motorOverHeating()){
            System.err.println("WARNING:"+MotorIdentification+":Motor is over heating");
        }
      
    }

}