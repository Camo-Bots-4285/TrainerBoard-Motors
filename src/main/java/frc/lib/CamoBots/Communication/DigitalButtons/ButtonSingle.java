package frc.lib.CamoBots.Communication.DigitalButtons;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * ButtonSingle
 * 
 * Utility class for managing a single toggle button on a NetworkTable.
 * 
 * Features:
 * - Creates a boolean button entry in a specified NetworkTable.
 * - Provides a Trigger for monitoring the button state.
 * - Allows programmatic setting of the button state.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class ButtonSingle {

    private final NetworkTableEntry dashboardEntry;  // NetworkTable entry for the button
    private final boolean defaultValue;               // Default button state

    /**
     * Constructs a ButtonSingle on the given NetworkTable with a default value.
     * 
     * @param table        NetworkTable to hold the button entry
     * @param buttonName   Name/key for the button entry
     * @param defaultValue Initial boolean value for the button
     */
    public ButtonSingle(NetworkTable table, String buttonName, boolean defaultValue) {
        this.dashboardEntry = table.getEntry(buttonName);
        this.dashboardEntry.setBoolean(defaultValue);
        this.defaultValue = defaultValue;
    }

    /**
     * Returns the NetworkTableEntry associated with this button.
     * 
     * @return NetworkTableEntry for direct access or reading
     */
    public NetworkTableEntry getEntry() {
        return dashboardEntry;
    }

    /**
     * Returns a Trigger that is active when the button's state is true.
     * Useful for command bindings.
     * 
     * @return Trigger that tracks the button's boolean state
     */
    public Trigger getDashboardEntryAsTrigger() {
        return new Trigger(() -> dashboardEntry.getBoolean(defaultValue));
    }

    /**
     * Programmatically sets the button's boolean state.
     * 
     * @param state true to activate the button, false to deactivate
     */
    public void setButtonState(boolean state) {
        dashboardEntry.setBoolean(state);
    }
}
