package frc.robot.lib.Thrifty_Hardware;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;


public class Thrifty_LaserCAN extends SubsystemBase {

    private static LaserCan m_LaserCAN;
    private static LaserCan.RangingMode Range;
    private static double Blocked_Distance;
    private static LaserCan.TimingBudget ObservationTimingBudgetValue=LaserCan.TimingBudget.TIMING_BUDGET_33MS;
      
        public Thrifty_LaserCAN(
            int CAN_ID,
            Boolean ShortRange,
            int[] Observation_Area,
            int ObservationTimingBudget,
            double Blocked_Distance
        ){
          CanBridge.runTCP();
          this.Blocked_Distance=Blocked_Distance;
          if(ShortRange==true){Range=LaserCan.RangingMode.SHORT;}else{Range=LaserCan.RangingMode.LONG;}
          if(ObservationTimingBudget==0){ObservationTimingBudgetValue=LaserCan.TimingBudget.TIMING_BUDGET_20MS;}
          else if(ObservationTimingBudget==1){ObservationTimingBudgetValue=LaserCan.TimingBudget.TIMING_BUDGET_33MS;}
          else if(ObservationTimingBudget==2){ObservationTimingBudgetValue=LaserCan.TimingBudget.TIMING_BUDGET_50MS;}
          else if(ObservationTimingBudget==3){ObservationTimingBudgetValue=LaserCan.TimingBudget.TIMING_BUDGET_100MS;}
          //Defines the Can_ID
          m_LaserCAN = new LaserCan(CAN_ID);

        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
        // GrappleHook
        try {//https://docs.thethriftybot.com/lasercan/configure-with-grapplehook
        /*This will define if we are in Long range(4m max) or Short range(1.3m max) */
          m_LaserCAN.setRangingMode(Range);

        /*This will define the range of intrest in a 16 by 16 grid with the center being (8,8) 
         * LaserCan.RegionOfInterest(x, y, X_range, Y-Range) for example LaserCan.RegionOfInterest(8, 8, 1, 16) will form a line on center
        */
          m_LaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(Observation_Area[0], Observation_Area[1], Observation_Area[2], Observation_Area[3]));

        /*This will define how long the laser can will run before giving a measurement long is more acruate but slowe */
          m_LaserCAN.setTimingBudget(ObservationTimingBudgetValue);

        } catch (ConfigurationFailedException e) {
          //System.out.println("Configuration failed! " + e);
        }
    
        }
    
        public static double getDistance_cm (){//Get the distance in mm and divide by ten to give cm
            LaserCan.Measurement measurement1 = m_LaserCAN.getMeasurement();
            return measurement1.distance_mm / 10;
        }
        
        public static Boolean getBlocked() {//retrun boolean if the measurent is less then the given

            if (getDistance_cm () <= Blocked_Distance) {
                return true;
              } else {
                return false;
              }
        }   
}


