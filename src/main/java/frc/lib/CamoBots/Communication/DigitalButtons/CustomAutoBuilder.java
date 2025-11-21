package frc.lib.CamoBots.Communication.DigitalButtons;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

/**
 * CustomAutoBuilder
 * 
 * Manages a group of selectable buttons on a dashboard for building custom autonomous routines.
 * 
 * Key Features:
 * - Tracks multiple button presses in order
 * - Built-in Reset, Back, Confirm, and Finished control buttons
 * - Designed for NetworkTables (Shuffleboard, AdvantageScope, etc.)
 * 
 * Useful for creating custom paths or command sequences based on user input.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class CustomAutoBuilder extends SubsystemBase {
    private final NetworkTableEntry[] buttons;     // User-selectable buttons
    private final String[] buttonNames;            // Names of selectable buttons
    private final List<Integer> activeButtonIndices = new ArrayList<>(); // Confirmed button sequence

    // Control buttons
    private final NetworkTableEntry resetButton;
    private final NetworkTableEntry backButton;
    private final NetworkTableEntry confirmButton;
    private final NetworkTableEntry finishedButton;

    // Previous states for edge detection on control buttons
    private boolean lastReset = false;
    private boolean lastBack = false;
    private boolean lastConfirm = false;
    private boolean lastFinished = false;

    private boolean finishedLatched = false;  // Latch for Finished button
    private boolean updated = false;           // Tracks if selection changed
    private Integer currentSelection = null;  // Currently selected button (not confirmed)

    /**
     * Constructs the CustomAutoBuilder with buttons registered on the given NetworkTable.
     * 
     * @param table The NetworkTable where buttons and controls are created.
     * @param buttonNames The list of button names (keys) for user selections.
     */
    public CustomAutoBuilder(NetworkTable table, String[] buttonNames) {
        this.buttonNames = buttonNames;
        this.buttons = new NetworkTableEntry[buttonNames.length];

        for (int i = 0; i < buttonNames.length; i++) {
            buttons[i] = table.getEntry(buttonNames[i]);
            buttons[i].setBoolean(false);
        }

        resetButton = table.getEntry("Reset");
        backButton = table.getEntry("Back");
        confirmButton = table.getEntry("Confirm");
        finishedButton = table.getEntry("Finished");

        resetButton.setBoolean(false);
        backButton.setBoolean(false);
        confirmButton.setBoolean(false);
        finishedButton.setBoolean(false);
    }

    /**
     * Periodically checks button states for user interaction and updates selections.
     */
    public void periodic() {
        // Detect new selection among user buttons
        for (int i = 0; i < buttons.length; i++) {
            boolean pressed = buttons[i].getBoolean(false);
            if (pressed && (currentSelection == null || currentSelection != i)) {
                selectOnlyThisButton(i);
                Logger.recordOutput("CustomAuto/Selected", buttonNames[i]);
                break;
            }
        }

        // Handle Reset button press
        boolean resetNow = resetButton.getBoolean(false);
        if (resetNow && !lastReset) {
            deselectAll();
            resetButton.setBoolean(true);
            reset();
            Logger.recordOutput("CustomAuto/Reset", true);
            resetButton.setBoolean(false);
        }
        lastReset = resetNow;

        // Handle Back button press
        boolean backNow = backButton.getBoolean(false);
        if (backNow && !lastBack) {
            deselectAll();
            backButton.setBoolean(true);
            back();
            Logger.recordOutput("CustomAuto/BackRemoved", getLastRemovedName());
            backButton.setBoolean(false);
        }
        lastBack = backNow;

        // Handle Confirm button press
        boolean confirmNow = confirmButton.getBoolean(false);
        if (confirmNow && !lastConfirm) {
            deselectAll();
            confirmButton.setBoolean(true);
            confirmSelection();
            Logger.recordOutput("CustomAuto/Confirmed", getSelectedButtonNames());
            confirmButton.setBoolean(false);
        }
        lastConfirm = confirmNow;

        // Handle Finished button press
        boolean finishedNow = finishedButton.getBoolean(false);
        if (finishedNow && !lastFinished) {
            deselectAll();
            finishedButton.setBoolean(true);
            Logger.recordOutput("CustomAuto/FinishedTriggered", true);
            finishedLatched = true;
            finishedButton.setBoolean(false);
        }
        lastFinished = finishedNow;

        // Log the path sequence for debug
        Logger.recordOutput("CustomAuto/PathSequence", buildPathSequence(getSelectedButtonNames()).toString());
        SmartDashboard.putString("CustomAuto/PathSequence", buildPathSequence(getSelectedButtonNames()).toString());
    }

    /**
     * Selects the specified button exclusively.
     * 
     * @param index Index of the button to select.
     */
    private void selectOnlyThisButton(int index) {
        if (currentSelection != null && currentSelection != index) {
            buttons[currentSelection].setBoolean(false);
        }
        currentSelection = index;
        buttons[currentSelection].setBoolean(true);
    }

    /**
     * Confirms the current selection and adds it to the active sequence.
     */
    private void confirmSelection() {
        if (currentSelection != null) {
            activeButtonIndices.add(currentSelection);
            updated = true;
        }
    }

    /**
     * Clears all selections and resets internal state.
     */
    public void reset() {
        updated = true;
        deselectAll();
        activeButtonIndices.clear();
        currentSelection = null;
        finishedLatched = false;
    }

    /**
     * Removes the last confirmed selection (undo) and updates current selection accordingly.
     */
    public void back() {
        updated = true;
        if (!activeButtonIndices.isEmpty()) {
            int last = activeButtonIndices.remove(activeButtonIndices.size() - 1);
            buttons[last].setBoolean(false);

            if (!activeButtonIndices.isEmpty()) {
                int previous = activeButtonIndices.get(activeButtonIndices.size() - 1);
                selectOnlyThisButton(previous);
            } else {
                currentSelection = null;
            }
        }
    }

    /**
     * Deselects all user and control buttons.
     */
    private void deselectAll() {
        for (NetworkTableEntry button : buttons) {
            button.setBoolean(false);
        }
        resetButton.setBoolean(false);
        backButton.setBoolean(false);
        confirmButton.setBoolean(false);
        finishedButton.setBoolean(false);
    }

    /**
     * Returns the name of the last confirmed button or "None" if none.
     * 
     * @return Last confirmed button name or "None".
     */
    private String getLastRemovedName() {
        if (activeButtonIndices.isEmpty()) {
            return "None";
        }
        int lastIndex = activeButtonIndices.get(activeButtonIndices.size() - 1);
        return buttonNames[lastIndex];
    }

    /**
     * Returns an array of all confirmed button names in sequence order.
     * 
     * @return Array of confirmed button names.
     */
    public String[] getSelectedButtonNames() {
        return activeButtonIndices.stream()
                .map(i -> buttonNames[i])
                .toArray(String[]::new);
    }

    /**
     * Returns whether a button (by index) is currently active.
     * 
     * @param index Index of the button to check.
     * @return True if active, false otherwise.
     */
    public boolean isButtonActive(int index) {
        return buttons[index].getBoolean(false);
    }

    /**
     * Constructs a list of transitions between consecutive steps.
     * Useful for visualizing the path sequence.
     * 
     * @param steps Array of step names.
     * @return List of transitions as strings.
     */
    public static List<String> buildPathSequence(String[] steps) {
        List<String> result = new ArrayList<>();
        for (int i = 0; i < steps.length - 1; i++) {
            result.add(steps[i] + "-" + steps[i + 1]);
        }
        return result;
    }

    /**
     * Returns true if there has been a change to the selection.
     * 
     * @return True if updated since last check.
     */
    public boolean getUpdated() {
        return updated;
    }

    /**
     * Acknowledges the latest update, clearing the update flag.
     */
    public void acknowledgeUpdate() {
        updated = false;
    }

    /**
     * Returns true if the Finished button has been triggered (latched).
     * 
     * @return True if finished, false otherwise.
     */
    public boolean isFinished() {
        return finishedLatched;
    }
}
