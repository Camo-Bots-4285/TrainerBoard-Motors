package frc.robot.HumanInterface.StateMachine;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SingleMotor.SingleSetSpeed;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

/**
 * State_Single:
 * 
 * A concrete implementation of State for controlling a single Neo_Single subsystem.
 * Only Button 1 triggers an action; others do nothing by default.
 */
public class State_Single extends State_Template {

    //private Supplier<Command> longcommand;

    public State_Single(StateManager stateManager, FlyWheelSubsystem m_flyWheel, ClimberSubsystem m_climber) {
        super(stateManager, m_flyWheel, m_climber);

        //this.longcommand=()-> new ToPose(0, 0, 0);
    }

    @Override
    public String Current_State() {
        return "Single state"; // State name
    }

    @Override
    public Supplier<Command> OnSwitchCommand() {
        return () -> Commands.none(); // No action on switch
    }

    @Override
    public Supplier<Command> Button_1() {
        return  ()-> new SingleSetSpeed(m_flyWheel); // Run 'One' command
    }

    @Override
    public Supplier<Command> Button_2() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_3() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_4() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_5() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_6() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_7() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_8() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_9() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_10() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_11() {
        return () -> Commands.none(); // No command
    }

    @Override
    public Supplier<Command> Button_12() {
        return () -> Commands.none(); // No command
    }
}
