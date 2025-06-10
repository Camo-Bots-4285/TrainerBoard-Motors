package frc.robot.control.StateMachine;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Neo_Single;


/**
 * StateManager
 * <p>
 * Description:
 * This class will be the main interface to control the state machine. It set the default state, 
 * allows tracking of the current state, and allows changeing of the current state.
 * <p>
 * Notes: This will change from year to year<
 * <p>
 * TODO: Following the current template to make extension of State
 */
public class StateManager {

    /**Keep the current state */
    private State state;


    /**StateManager - Constructor
     * <p>
     * Passes thought subsystem to State extensions and set the state at the begining
      */
    public StateManager (Neo_Single m_single_neo){
        setState(new State_Double(this,m_single_neo));
    }


    /**Function: Return the current state which can be used the acess the method with that class */
    public State getState(){
       return state;
    }

    
    /**  Track all commands associated with the current state*/
    private final List<Command> activeCommands = new ArrayList<>();


    /** Function: Change the state
     * <p>
     * Order: Cancels command that are to be cleared -> Set new state -> Define new command to be clear -> Runs intial commands for state
     * @param newState The new State extends that the state should be switched to
     */
    public void setState(State newState){
        // Cancel any commands that were running from the previous state
        for (Command cmd : activeCommands) {
            if (cmd != null && cmd.isScheduled()) {
                cmd.cancel();
            }
        }

        // Set new state
        this.state = newState;

        // Optional: run the state's on-switch logic
        Command onSwitch = newState.OnSwitchCommand();
        if (onSwitch != null && onSwitch != Commands.none()) {
            onSwitch.schedule();
        }

        // Preload all new commands into the list for possible future canceling
        activeCommands.clear();
        activeCommands.add(newState.Button_1());
        activeCommands.add(newState.Button_2());
        activeCommands.add(newState.Button_3());
        activeCommands.add(newState.Button_4());
        activeCommands.add(newState.Button_5());
        activeCommands.add(newState.Button_6());
        activeCommands.add(newState.Button_7());
        activeCommands.add(newState.Button_8());
        activeCommands.add(newState.Button_9());
        activeCommands.add(newState.Button_10());
        activeCommands.add(newState.Button_11());
        activeCommands.add(newState.Button_12());

        //Runs the new commands on switch 
        newState.OnSwitchCommand();

    }
 
}
