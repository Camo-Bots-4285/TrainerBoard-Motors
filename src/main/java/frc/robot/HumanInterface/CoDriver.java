package frc.robot.HumanInterface;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SingleMotor.One;
import frc.robot.commands.SingleMotor.SingleSetSpeed;
import frc.robot.commands.SingleMotor.Zero;
import frc.robot.lib.Communication.DigitalButtons.ButtonGroup;
import frc.robot.lib.Communication.DigitalButtons.CustomAutoBuilder;
import frc.robot.lib.Helpers.CommandHelper;
import frc.robot.subsystems.FlyWheelSubsystem;


/**
 * CoDriver:
 * 
 * Manages co-driver touchscreen buttons and their command bindings.
 * Supports controlling the Neo_Single motor subsystem, vision systems, and more.
 * 
 * Buttons are grouped and tied to NetworkTables entries for dashboard control.
 */
public class CoDriver extends SubsystemBase {
    /**
     * Constructor
     * 
     * @param m_flyWheel Neo_Single motor subsystem
     */
    public CoDriver(FlyWheelSubsystem m_flyWheel) {
        this.m_flyWheel = m_flyWheel;
    }

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    // Subsystems
    private FlyWheelSubsystem m_flyWheel;


    // NetworkTable for co-driver digital buttons
    public NetworkTable table = networkTableInstance.getTable("CoDriver_Digital_Buttons");

    // Grouped buttons for motor commands
    private String[] Motor1_Buttons_Names = { "Zero", "One", "Spin" };
    public ButtonGroup Motor1_ButtonGroup = new ButtonGroup(table, Motor1_Buttons_Names);

    //Grouped button to help make custom auto
    private String[] AutoMaker_Names = { "1", "2", "3","4" };
    public CustomAutoBuilder AutoMaker = new CustomAutoBuilder(table, AutoMaker_Names);
     
    //Allows robot to access CostomAutoBuilder 
    public CustomAutoBuilder getAutoMaker() {
        return AutoMaker;
    }



    /**
     * Sets up button bindings for co-driver interface buttons.
     */
    public void CoDriverBindings() {
        // Bind Motor1 buttons to commands
        Trigger btn_Zero = Motor1_ButtonGroup.getDashboardEntryAsTrigger(0);
        btn_Zero.onTrue(CommandHelper.returnIfValid(() -> new Zero(m_flyWheel)));

        Trigger btn_One = Motor1_ButtonGroup.getDashboardEntryAsTrigger(1);
        btn_One.onTrue(CommandHelper.returnIfValid(() -> new One(m_flyWheel)));

        Trigger btn_Spin = Motor1_ButtonGroup.getDashboardEntryAsTrigger(2);
        btn_Spin.onTrue(CommandHelper.returnIfValid(() -> new SingleSetSpeed(m_flyWheel)));

    }

    /**
     * Checks button states and schedules commands accordingly.
     * Call periodically if needed.
     */
    public void CoDriverIntCommands() {

    
    }
}
