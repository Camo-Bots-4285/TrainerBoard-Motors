package frc.robot.control;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.lib.Communication.ButtonGroup;
import frc.robot.subsystems.Neo_Single;

/**
 * TunningInterface
 * <p>
 * Description:
 * This class will hold all button that should be mapped to SysID which will assist tunning
 * <p>
 * Notes:This should never be running while the robot is commeting
 * <p>
 * TODO: Change the subsystem to tune something else
 */
public class TunningInterface {
    
    private Neo_Single m_single_neo;
    private NetworkTableInstance networkTableInstance=NetworkTableInstance.getDefault();

    /**
     * TunnerContainer hold a ButtonGroup that will run SysID routine
     */
    public TunningInterface(Neo_Single m_single_neo){
        this.m_single_neo = m_single_neo;
       
    }

    public NetworkTable table = networkTableInstance.getTable("TunningDigitalButtons");

    private String[] SysID_Buttons_Names= {"quasisatic_forward","quasisatic_reverse","dynamic_forward","dynamic_reverse"};
    public final ButtonGroup SysID_ButtonGroup = new ButtonGroup(table, SysID_Buttons_Names);

    /**
     * TunnerContainer hold a ButtonGroup that will run SysID routine
     */
    public void TunnerBindings(){

        if (m_single_neo != null) {//Check that m_single_neo is enabled

        //This will run the sysID routine for the motor selected
        Trigger btn_quasisatic_forward = SysID_ButtonGroup.getDashboardEntryAsTrigger(0);
        btn_quasisatic_forward.whileTrue(m_single_neo.Single_Motor.sysIdQuasistatic(Direction.kForward));

        Trigger btn_quasisatic_reverse = SysID_ButtonGroup.getDashboardEntryAsTrigger(1);
        btn_quasisatic_reverse.whileTrue(m_single_neo.Single_Motor.sysIdQuasistatic(Direction.kReverse));
           
        Trigger btn_dynamic_forward = SysID_ButtonGroup.getDashboardEntryAsTrigger(2);
        btn_dynamic_forward.whileTrue(m_single_neo.Single_Motor.sysIdDynamic(Direction.kForward));

        Trigger btn_dynamic_reverse = SysID_ButtonGroup.getDashboardEntryAsTrigger(3);
        btn_dynamic_reverse.whileTrue(m_single_neo.Single_Motor.sysIdDynamic(Direction.kReverse));

        }

    }
    
}
