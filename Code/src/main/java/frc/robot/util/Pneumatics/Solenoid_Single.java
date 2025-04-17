package frc.robot.util.Pneumatics;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Solenoid_Single extends SubsystemBase {//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/solenoids.html

    private final Solenoid m_singleSolenoid;
      
        public Solenoid_Single(
        int pneumatic_Hub,
        int pneumatic_Channel
        ){

        m_singleSolenoid= new Solenoid(
        pneumatic_Hub,
        PneumaticsModuleType.REVPH,
        pneumatic_Channel
        );

}
    
public void setSoleniodPosition(boolean extended){
    m_singleSolenoid.set(extended);
}

public void setSoleniodPulse(double Time_0_00x){
    m_singleSolenoid.setPulseDuration(Time_0_00x);
    m_singleSolenoid.startPulse();
}

}


