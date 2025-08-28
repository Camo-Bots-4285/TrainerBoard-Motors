package frc.robot.lib.Communication.DigitalButtons;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * ButtonGroup
 * 
 * Manages a group of digital buttons on a NetworkTable UI.
 * Ensures only one button in the group can be active at a time (mutual exclusivity).
 * 
 * Useful for mode selectors, autonomous routine choices, or toggle behaviors.
 * 
 * Features:
 * - Creates NetworkTable entries for each button
 * - Tracks and updates active button state efficiently
 * - Uses event-driven approach to avoid constant polling overhead
 * - Provides Triggers for command bindings
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class ButtonGroup extends SubsystemBase {

    private final NetworkTableEntry[] buttons;    // Button entries on NetworkTables
    private final String[] buttonNames;           // Corresponding button names
    private int lastActiveIndex = -1;             // Index of currently active button

    /**
     * Constructs a ButtonGroup using the given NetworkTable and button names.
     * Initializes all buttons to false except the first which is enabled by default.
     * 
     * @param table        NetworkTable where buttons will be stored
     * @param buttonNames  Names of the buttons for keys in the table
     */
    public ButtonGroup(NetworkTable table, String[] buttonNames) {
        this.buttonNames = buttonNames;
        buttons = new NetworkTableEntry[buttonNames.length];

        // Initialize buttons as NetworkTable entries
        for (int i = 0; i < buttonNames.length; i++) {
            buttons[i] = table.getEntry(buttonNames[i]);
            buttons[i].setBoolean(false);
        }

        // Enable first button by default
        if (buttons.length > 0) {
            buttons[0].setBoolean(true);
            lastActiveIndex = 0;
        }
    }

    /**
     * Returns index of the currently active button.
     * 
     * @return active button index or -1 if none active
     */
    public int getActiveButtonIndex() {
        return lastActiveIndex;
    }

    /**
     * Returns name of the currently active button.
     * 
     * @return active button name or "None" if none active
     */
    public String getActiveButtonName() {
        if (lastActiveIndex >= 0 && lastActiveIndex < buttonNames.length) {
            return buttonNames[lastActiveIndex];
        }
        return "None";
    }

    /**
     * Returns a Trigger that is true when the button at index i is active.
     * Useful for command bindings.
     * 
     * @param i index of the button
     * @return Trigger representing button state
     */
    public Trigger getDashboardEntryAsTrigger(int i) {
        return new Trigger(() -> buttons[i].getBoolean(false));
    }

    /**
     * Checks if any button's state has changed to active (true),
     * different from the last active button.
     * 
     * @return index of new active button or -1 if no change
     */
    private int getChangedButtonIndex() {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i].getBoolean(false) && i != lastActiveIndex) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Deactivates all buttons except the one at activeIndex.
     * 
     * @param activeIndex index of the button to keep active
     */
    private void resetOtherButtons(int activeIndex) {
        for (int i = 0; i < buttons.length; i++) {
            buttons[i].setBoolean(i == activeIndex);
        }
    }

    /**
     * Periodically called by the scheduler.
     * Detects changes to active button and updates button states accordingly.
     * Ensures only one button remains active at any time.
     * Uses event-driven logic to minimize unnecessary writes.
     */
    @Override
    public void periodic() {
        int newActiveIndex = getChangedButtonIndex();

        if (newActiveIndex != -1 && newActiveIndex != lastActiveIndex) {
            resetOtherButtons(newActiveIndex);
            lastActiveIndex = newActiveIndex;
        }
    }
}
