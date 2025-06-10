package frc.robot.control.StateMachine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Neo_Single;

/**
 * State
 * <p>
 * Description:
 * Is an abstract class, like a template that get filled that changes based which state you fill it in with.
 * The states will pass thought different command depending on the state selected.
 * <p>
 * Notes: This will change from year to year
 * <p>
 * TODO: Following the current template to make extension of State
 */
public abstract class State {

    //Define Subsystem to be pass thought to the bastract class
    protected final StateManager stateManager;
    protected final Neo_Single m_single_neo;

    /**Passes thought dependencies that can be used later in commands
     * <p>
    * Example-Subsystems and state manager (Subsystem m_subsystem)
    */
    public State(StateManager stateManager, Neo_Single m_single_neo){
        this.m_single_neo=m_single_neo;
        this.stateManager=stateManager;
    }

    /**Returns a String with the name of the current state */
    public abstract String Current_State();
    /**This command will be called when the state switches after whip to insure do old commands
     * are left on. */
    public abstract Command OnSwitchCommand();

    /**Command will be mapped to Button 1 on the driver joystick */
    public abstract Command Button_1();
    /**Command will be mapped to Button 2 on the driver joystick */
    public abstract Command Button_2();
    /**Command will be mapped to Button 3 on the driver joystick */
    public abstract Command Button_3();
    /**Command will be mapped to Button 4 on the driver joystick */
    public abstract Command Button_4();
    /**Command will be mapped to Button 5 on the driver joystick */
    public abstract Command Button_5();
    /**Command will be mapped to Button 6 on the driver joystick */
    public abstract Command Button_6();
    /**Command will be mapped to Button 7 on the driver joystick */
    public abstract Command Button_7();
    /**Command will be mapped to Button 8 on the driver joystick */
    public abstract Command Button_8();
    /**Command will be mapped to Button 9 on the driver joystick */
    public abstract Command Button_9();
    /**Command will be mapped to Button 10 on the driver joystick */
    public abstract Command Button_10();
    /**Command will be mapped to Button 11 on the driver joystick */
    public abstract Command Button_11();
    /**Command will be mapped to Button 12 on the driver joystick */
    public abstract Command Button_12();


}
