package frc.robot.lib.Communication;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * ButtonGroup
 * 
 * Description:
 * This class allows management and interaction with a group of buttons in the NetworkTable.
 * It provides functionality for monitoring and changing the states of multiple buttons at once, ensuring that only one button
 * in the group is active at any given time. This can be useful in scenarios where multiple options are available, but only one
 * should be selected at a time.
 * 
 * Methods:
 * - getActiveButtonIndex: Returns the index of the currently active button.
 * - getActiveButtonName: Returns the name of the currently active button.
 * - getDashboardEntryAsTrigger: Returns a Trigger for the specified button in the group, based on the button's state.
 * - periodic: Periodically checks the state of the buttons to ensure only one button is active at a time.
 * 
 * Notes:
 * - The class is designed to manage multiple buttons and their states. Only one button can be active at any time.
 * - The buttons' states are stored and updated in the NetworkTable.
 * - Ensure the NetworkTable is properly initialized before using this class.
 * 
 * TODO: Test NetworkTable verions and remove Shuffleboard version on success
 */


public class ButtonGroup extends SubsystemBase {
    // private int lastActiveIndex = -1; // Track last active button
    // private GenericEntry[] buttons;
    // private String[] ButtonNames;

    // public ButtonGroup(ShuffleboardTab tab, String[] ButtonNames) {
    //     this.ButtonNames=ButtonNames;

    //     for (int i = 0; i < ButtonNames.length; i++) {
    //         // ShuffleboardTab tab = Shuffleboard.getTab("Controls-Arm");
    //         buttons[i] = tab.add(ButtonNames[i], false).withWidget("Toggle Button").getEntry();
    //     }

    //     for (int i = 0; i < buttons.length; i++) {
    //         if (buttons[i] != null) {
    //             buttons[i].setBoolean(true); // Set first available button ON
    //             lastActiveIndex = i;
    //             break;
    //         }
    //     }
    // }

    // private int getChangedButtonIndex() {
    //     for (int i = 0; i < buttons.length; i++) {
    //         if (buttons[i] != null) {
    //             boolean currentState = buttons[i].getBoolean(false);
    //             if (currentState && i != lastActiveIndex) { // Detect new button press
    //                 return i;
    //             }
    //         }
    //     }
    //     return -1;
    // }

    // private void resetOtherButtons(int activeIndex) {
    //     for (int i = 0; i < buttons.length; i++) {
    //         if (buttons[i] != null) {
    //             buttons[i].setBoolean(i == activeIndex); // Keep only one button active
    //         }
    //     }
    // }

    // public int getActiveButtonIndex() {
    //     return lastActiveIndex;
    // }

    // public String getActiveButtonName() {
    //     if (lastActiveIndex >= 0 && lastActiveIndex < ButtonNames.length) {
    //         return ButtonNames[lastActiveIndex];
    //     }
    //     return "None";
    // }

    // public Trigger getDashboardEntryAsTrigger(int i){
    //   return new Trigger(()-> buttons[i].getBoolean(false));
    // }

    // @Override
    // public void periodic() {
    //     int newActiveIndex = getChangedButtonIndex();

    //     if (newActiveIndex != -1 && newActiveIndex != lastActiveIndex) {
    //         resetOtherButtons(newActiveIndex);

    //         if (buttons[newActiveIndex] != null) {
    //             buttons[newActiveIndex].setBoolean(true);
    //             lastActiveIndex = newActiveIndex;
    //         }
    //     }
    // }


    private int lastActiveIndex = -1; // Track last active button
    private NetworkTableEntry[] buttons;
    private String[] ButtonNames;
    

    public ButtonGroup(NetworkTable table, String[] ButtonNames) {
        this.ButtonNames = ButtonNames;
        buttons = new NetworkTableEntry[ButtonNames.length];

        // Initialize buttons with NetworkTable entries
        for (int i = 0; i < ButtonNames.length; i++) {
            buttons[i] = table.getEntry(ButtonNames[i]); // Create entry for each button
            buttons[i].setBoolean(false); // Initialize button state as false (not pressed)
        }

        // Set the first button to true (active) by default
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                buttons[i].setBoolean(true); // Set first available button ON
                lastActiveIndex = i;
                break;
            }
        }
    }

    private int getChangedButtonIndex() {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                boolean currentState = buttons[i].getBoolean(false);
                if (currentState && i != lastActiveIndex) { // Detect new button press
                    return i;
                }
            }
        }
        return -1;
    }

    private void resetOtherButtons(int activeIndex) {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                buttons[i].setBoolean(i == activeIndex); // Keep only one button active
            }
        }
    }

    public int getActiveButtonIndex() {
        return lastActiveIndex;
    }

    public String getActiveButtonName() {
        if (lastActiveIndex >= 0 && lastActiveIndex < ButtonNames.length) {
            return ButtonNames[lastActiveIndex];
        }
        return "None";
    }

    public Trigger getDashboardEntryAsTrigger(int i){
      return new Trigger(()-> buttons[i].getBoolean(false));
    }

    @Override
        public void periodic() {
 
        int newActiveIndex = getChangedButtonIndex();

        if (newActiveIndex != -1 && newActiveIndex != lastActiveIndex) {
            resetOtherButtons(newActiveIndex);

            if (buttons[newActiveIndex] != null) {
                buttons[newActiveIndex].setBoolean(true);
                lastActiveIndex = newActiveIndex;
            }
        }

        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                boolean currentState = buttons[i].getBoolean(false);
                Logger.recordOutput("Communication/DigitalButtons/Display/"+ButtonNames[i],currentState);
            }
        }

        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                boolean currentState = getDashboardEntryAsTrigger(i).getAsBoolean();
                Logger.recordOutput("Communication/DigitalButtons/Trigger/"+ButtonNames[i],currentState);
            }
        }
    }
}
