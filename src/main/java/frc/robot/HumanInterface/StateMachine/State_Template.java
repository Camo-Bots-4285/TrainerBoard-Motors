package frc.robot.HumanInterface.StateMachine;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

/**
 * State:
 * 
 * This is an abstract template for different robot states.
 * Each state will define commands tied to joystick buttons and actions when switching states.
 * 
 * <p><b>Why use this?</b>
 * It lets you create many different modes (states) for the robot, each with its own button mappings and logic.
 * 
 * <p><b>How it works:</b>
 * Subclasses implement each method to provide commands for buttons and what happens on switching to that state.
 * 
 * <p><b>Dependencies:</b>
 * Passes in subsystems and the StateManager to share across commands.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public abstract class State_Template {

    protected final StateManager stateManager;  // Reference to StateManager for state control
    protected final FlyWheelSubsystem m_flyWheel;    // Example subsystem passed into the state
    protected final ClimberSubsystem m_climber;  

    /** Constructor to pass dependencies like subsystems and state manager */
    public State_Template(StateManager stateManager, FlyWheelSubsystem m_flyWheel, ClimberSubsystem m_climber){
        this.m_flyWheel = m_flyWheel;
        this.stateManager = stateManager;
        this.m_climber = m_climber;
    }

    /** Returns the name of the current state */
    public abstract String Current_State();

    /** Command to run immediately when switching to this state (e.g., cleanup or setup) */
    public abstract Supplier<Command> OnSwitchCommand();

    // Commands mapped to joystick buttons 1 through 12
    public abstract Supplier<Command> Button_1();
    public abstract Supplier<Command> Button_2();
    public abstract Supplier<Command> Button_3();
    public abstract Supplier<Command> Button_4();
    public abstract Supplier<Command> Button_5();
    public abstract Supplier<Command> Button_6();
    public abstract Supplier<Command> Button_7();
    public abstract Supplier<Command> Button_8();
    public abstract Supplier<Command> Button_9();
    public abstract Supplier<Command> Button_10();
    public abstract Supplier<Command> Button_11();
    public abstract Supplier<Command> Button_12();

}

