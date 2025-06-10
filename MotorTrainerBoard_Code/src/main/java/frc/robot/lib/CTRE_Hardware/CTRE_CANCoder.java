package frc.robot.lib.CTRE_Hardware;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class CTRE_CANCoder {

  private final CANcoder canCoder;
  private double RadianOffSet;
    public CTRE_CANCoder(
        int CANCoderID,
        double RadianOffSet,
        String CANCoderCANLoop
    ){
        this.RadianOffSet=RadianOffSet;
        // configure the CANCoder to output in unsigned (wrap around from 360 to 0
    // degrees)
   


    // Pretty sure we need to to change the method compared to v5 -- Fixed
    //canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // Initialize the CANcoder with the specified CAN ID
        canCoder = new CANcoder(CANCoderID, CANCoderCANLoop);

        //Concoder from CTRE migraiton doecs
        // // Configure the CANcoder
        // CANcoderConfiguration config = new CANcoderConfiguration();

        // // Set sensor direction (false = default direction, true = inverted)
        // config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


    MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
    //This line is the one making the sencor wrap around at one
    magnetConfig.withAbsoluteSensorDiscontinuityPoint(1);
    
    // Apply the configuration to make the CANCoder wrap around from 360 to 0 degrees
    canCoder.getConfigurator().apply(magnetConfig);
    }

    public Rotation2d getRawAngle(){
        return Rotation2d.fromRotations(canCoder.getPosition().getValueAsDouble());
    }

    public Rotation2d getOptimzedAngle() {
    //Changed to this to Assume the Cancode is giving out rotations
     double unsignedAngle = (Units.rotationsToRadians(canCoder.getPosition().getValueAsDouble()) - RadianOffSet)
         % (2 * Math.PI);

    return new Rotation2d(unsignedAngle);
    }

  public void setPosition (double New_Position){
    canCoder.setPosition(New_Position);
  }
}
