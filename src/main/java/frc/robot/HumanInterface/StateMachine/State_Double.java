package frc.robot.HumanInterface.StateMachine;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Exampled.SetSpeed;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

/**
 * State_Double:
 * 
 * Concrete State implementation controlling a Neo_Single subsystem.
 * Defines commands triggered by joystick buttons in this "Double" state.
 * 
 * Currently, only Button 1 triggers a command; all other buttons do nothing.
 * 
 * Extend or customize this class to add more button commands or behaviors.
 */
public class State_Double extends State_Template {

    public State_Double(StateManager stateManager, FlyWheelSubsystem m_flyWheel, ClimberSubsystem m_climber) {
        super(stateManager, m_flyWheel, m_climber);
    }

    @Override
    public String Current_State() {
        return "Double state";  // Returns the name of this state
    }

    @Override
    public Supplier<Command> OnSwitchCommand() {
        return () -> Commands.none();  // No special action on state switch
    }

    @Override
    public Supplier<Command> Button_1() {
        // Return Zero command wrapped by helper to check validity
        return  ()-> new SetSpeed(m_flyWheel, m_climber);
    }

    // All other buttons currently have no associated commands and return no-ops
    @Override
    public Supplier<Command> Button_2() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_3() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_4() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_5() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_6() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_7() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_8() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_9() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_10() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_11() {
        return () -> Commands.none();
    }

    @Override
    public Supplier<Command> Button_12() {
        return () -> Commands.none();
    }

}
