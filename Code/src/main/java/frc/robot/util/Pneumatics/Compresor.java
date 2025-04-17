package frc.robot.util.Pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Compresor {//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pressure.html
      public static Compressor m_compressor;

      public  Compresor(
        int Compressor_Channel
        ){



        m_compressor = new Compressor(
        Compressor_Channel, 
        PneumaticsModuleType.REVPH
     );

        }

 public void runCompressor(){
    // Enable closed-loop mode based on the analog pressure sensor connected to the PH.
    // The compressor will run while the pressure reported by the sensor is in the
    // specified range ([102.5 PSI, 115 PSI] in this example).
    // Analog mode exists only on the PH! On the PCM, this enables digital control.
    m_compressor.enableAnalog(102.5, 115.0);
 }

 public void disableCompressor(){
    m_compressor.disable();
}

 public double getCurrentPressure(){
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    return m_compressor.getPressure();
 }


}
