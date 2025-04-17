package frc.robot.util.Pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Solenoid_Double extends SubsystemBase {//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/solenoids.html
    private DoubleSolenoid m_doubleSolenoid;
      
        public Solenoid_Double(
                int pneumatic_Hub,
                int pneumatic_Channel_On,
                int pneumatic_Channel_Off
                ){
    

                    m_doubleSolenoid = new DoubleSolenoid (
                    pneumatic_Hub,
                    PneumaticsModuleType.REVPH,
                    pneumatic_Channel_On,
                    pneumatic_Channel_Off
                    );

            }
                

            
    public void setSoleniodPosition(boolean extended){
        if(extended=true){
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        else{
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }



}
