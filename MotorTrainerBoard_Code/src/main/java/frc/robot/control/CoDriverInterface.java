package frc.robot.control;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.CommandHelper;
import frc.robot.lib.Communication.ButtonGroup;
import frc.robot.lib.Communication.ButtonSingle;
import frc.robot.lib.Communication.ElasticNotifier;
import frc.robot.lib.Communication.NetworkTable_Publishers.Boolean_Publisher;
import frc.robot.subsystems.Neo_Single;
import frc.robot.commands.SingleMotor.*;

/**
 * CoDriverInterface
 * <p>
 * Description:
 * This class will hold all button that should be displayed on the co-driver touch screen along with there bindings
 * <p>
 * Notes: This will change from year to year
 * <p>
 * TODO: Add more button and map them to commands
 */
public class CoDriverInterface extends SubsystemBase {


    private NetworkTableInstance networkTableInstance=NetworkTableInstance.getDefault();

    /**
     * CoDriverInterface - Constructor
     * <p>
     * Description:
     * Passes thought subsystem to be pass thought to commands
     */
    private Neo_Single m_single_neo;
        public CoDriverInterface(Neo_Single m_single_neo){
            this.m_single_neo = m_single_neo;
    
        }

    /*The following shows hoe to us button group to run command keep in mind you can also pull the index and string name 
     * to a subsytem if writng out commands for button is not oprimal. Group button are to be use when you have a group and you 
     * only want one button active at a time.
      */

     public NetworkTable table = networkTableInstance.getTable("CoDriver_Digital_Buttons");

     private String[] Motor1_Buttons_Names= {"Zero","One","Spin"};
     public final ButtonGroup Motor1_ButtonGroup = new ButtonGroup(table, Motor1_Buttons_Names);

    /**
     * CoDriverInterface - Bindings
     * <p>
     * Description:
     * Holds the button mapping for the CoDriverInterface to be played in RobotContainer for initation
     */
    public void CoDriverBindings(){

        Trigger btn_Zero = Motor1_ButtonGroup.getDashboardEntryAsTrigger(0);
        btn_Zero.onTrue(CommandHelper.returnIfValid(()-> new Zero(m_single_neo)));

        Trigger btn_One = Motor1_ButtonGroup.getDashboardEntryAsTrigger(1);
        btn_One.onTrue(CommandHelper.returnIfValid(()-> new One(m_single_neo)));

        Trigger btn_Spin = Motor1_ButtonGroup.getDashboardEntryAsTrigger(2);
        btn_Spin.onTrue(CommandHelper.returnIfValid(()-> new SingleSetSpeed(m_single_neo)));

    }
  
}
