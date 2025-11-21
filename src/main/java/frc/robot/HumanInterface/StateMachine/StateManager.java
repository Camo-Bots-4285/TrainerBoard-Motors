package frc.robot.HumanInterface.StateMachine;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CamoBots.Helpers.CommandHelper;

/**
 * StateManager:
 * 
 * This class controls which "State" the robot is currently in and manages transitions between states.
 * It cancels commands from the old state and starts commands associated with the new state.
 * 
 * <p><b>Why use this?</b> 
 * State machines help organize complex robot behavior into manageable modes, each with its own command sets.
 * This class makes switching between those modes clean and consistent.
 * 
 * <p><b>Key Features:</b>
 * - Tracks the current state.
 * - Cancels all active commands from the previous state when switching.
 * - Schedules commands linked to buttons for the new state.
 * - Runs any initialization commands defined by the new state on switch.
 * 
 * <p><b>Usage:</b>
 * When you want to change the robot’s mode (like from gamepieces to end game), call {@code setState(newState)}.
 * The class handles canceling old commands and starting new ones automatically.
 * 
 * <p><b>Important:</b> 
 * Make sure each State properly defines commands for buttons and any on-switch initialization.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class StateManager{

    /** The currently active state of the robot */
    private State_Template state;

    /** Constructor for StateManager */
    public StateManager() {
        // Initialization code if needed
    }

    /** Returns the current active state */
    public State_Template getState() {
        return state;
    }

    /**
     * Changes the current state to a new state.
     * 
     * <p>This method:
     * <ul>
     *   <li>Cancels any running commands from the previous state.</li>
     *   <li>Sets the new state.</li>
     *   <li>Schedules the new state's on-switch command, if any.</li>
     *   <li>Loads and tracks all commands tied to buttons for the new state.</li>
     * </ul>
     * 
     * @param newState the new state to switch to
     */
        public void setState(State_Template newState) {
            System.out.println("Switching to state: " + newState.getClass().toString());
            // assign the new state
            this.state = newState;
    }

    public enum BindType {
        WHILE_TRUE,
        ON_TRUE,
        TOGGLE_ON_TRUE
    }
    public <T extends State_Template> void bindAllButtonsForState(
        T stateInstance,
        JoystickButton[] buttons,
        BindType[] bindTypes
    ) {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] == null) {
                // Skip if button is null
                continue;
            }
    
            int index = i;
            Trigger stateTrigger = new Trigger(() -> getState() == stateInstance);
            Trigger combined = stateTrigger.and(buttons[i]);
    
            Supplier<Command> cmdSupplier = switch (index) {
                case 0 -> stateInstance.Button_1();
                case 1 -> stateInstance.Button_2();
                case 2 -> stateInstance.Button_3();
                case 3 -> stateInstance.Button_4();
                case 4 -> stateInstance.Button_5();
                case 5 -> stateInstance.Button_6();
                case 6 -> stateInstance.Button_7();
                case 7 -> stateInstance.Button_8();
                case 8 -> stateInstance.Button_9();
                case 9 -> stateInstance.Button_10();
                case 10 -> stateInstance.Button_11();
                case 11 -> stateInstance.Button_12();
                default -> () -> Commands.none();
            };
    
            //  Wrap the supplier with null and exception safety
            Supplier<Command> safeSupplier = CommandHelper.returnIfValidSupplier(cmdSupplier);

            // Bind command with desired trigger type
            switch (bindTypes[i]) {
                case WHILE_TRUE     -> combined.whileTrue(safeSupplier.get());
                case ON_TRUE        -> combined.onTrue(safeSupplier.get());
                case TOGGLE_ON_TRUE -> combined.toggleOnTrue(safeSupplier.get());
            }
        }
    }
    
 
}
