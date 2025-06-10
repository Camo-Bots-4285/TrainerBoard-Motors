package frc.robot.lib.Rev_Hardware.Sencors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 



public class REV_Sencor_Magnetic_Switch extends SubsystemBase{
    public DigitalInput m_magnetic_sencor; 

    public REV_Sencor_Magnetic_Switch(
        int Digital_Channel
    ){
        m_magnetic_sencor = new DigitalInput(Digital_Channel);
    }

    public boolean getState(){
    if (!m_magnetic_sencor.get())
    {
      return  m_magnetic_sencor.get();
    }
    else{System.err.println("Magnetic Sencor Null");return false;}
}
 
}

