package frc.robot.lib.Communication;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * ButtonSingle
 * 
 * Description:
 * This class provides a utility for creating and interacting with a button in a NetworkTable.
 * The button can be toggled, and its state can be monitored using triggers. It facilitates communication
 * between the robot and the human operator via the NetworkTable.
 * 
 * Methods:
 * - createButton: Creates a button in the specified NetworkTable with an initial state (default: false).
 * - getDashboardEntryAsTrigger: Retrieves a Trigger that evaluates based on the button's state in the NetworkTable.
 * - setButtonState: Allows setting the button's state programmatically from the robot code.
 * 
 * Notes:
 * - The button state can be toggled via the NetworkTable, and this class uses the default NetworkTable instance.
 * - Ensure NetworkTable is properly initialized and available before using this class.
 * 
 * TODO: Test NetworkTable verions and remove Shuffleboard version on success
 */


public class ButtonSingle extends SubsystemBase {

    // // ShuffleboardTab tab = Shuffleboard.getTab("Controls-Arm");
    // public static GenericEntry createButton(ShuffleboardTab tab, String Title){
    //     GenericEntry dashboardEntry = tab.add(Title, false).withWidget("Toggle Button").getEntry();
    //     return dashboardEntry;
    // }

    // public static Trigger getDashboardEntryAsTrigger(GenericEntry dashboardEntry){
    //   return new Trigger(()-> dashboardEntry.getBoolean(false));
    // }

    // public void setButtonState(GenericEntry dashboardEntry,boolean state){
    //     dashboardEntry.setBoolean(state);
    // }

    private NetworkTableEntry dashboardEntry;
    private boolean default_value;
    private String ButtonName;
    
      /**
     * Create a button entry in the NetworkTable
     * @param tableName The name of the NetworkTable to add the entry to
     * @param title The title or name of the button in the table
     * @return A NetworkTableEntry representing the button
     */
     public ButtonSingle(NetworkTable table, String ButtonName,boolean default_value) {
        // Add an entry to the table and initialize it with a boolean value (false by default)
        dashboardEntry = table.getEntry(ButtonName);
        dashboardEntry.setBoolean(default_value); // Default state
        this.default_value=default_value;
        this.ButtonName=ButtonName;
       
     }
    
  
    public NetworkTableEntry createButton() {
        return dashboardEntry;
    }

    /**
     * Get the button state as a Trigger from a NetworkTableEntry
     * @param dashboardEntry The entry representing the button
     * @return A Trigger that checks if the button is pressed
     */
    public Trigger getDashboardEntryAsTrigger() {
        return new Trigger(() -> dashboardEntry.getBoolean(default_value)); // Default is false if not set
        
    }

    /**
     * Set the button state in the NetworkTable
     * @param dashboardEntry The entry representing the button
     * @param state The state to set for the button (true or false)
     */
    public void setButtonState(NetworkTableEntry dashboardEntry, boolean state) {
        dashboardEntry.setBoolean(state); // Update the button state in the NetworkTable
    }

    @Override
    public void periodic() {
            boolean currentState_Display = dashboardEntry.getBoolean(default_value);
            Logger.recordOutput("Communication/DigitalButtons/Display/"+ButtonName,currentState_Display);

            boolean currentState_Trigger = getDashboardEntryAsTrigger().getAsBoolean();
            Logger.recordOutput("Communication/DigitalButtons/Trigger/"+ButtonName,currentState_Trigger);
        
    }

  
}



