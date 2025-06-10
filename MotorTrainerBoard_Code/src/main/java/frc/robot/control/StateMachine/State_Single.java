package frc.robot.control.StateMachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SingleMotor.One;
import frc.robot.lib.CommandHelper;
import frc.robot.subsystems.Neo_Single;


public class State_Single extends State {
    
    public State_Single(StateManager stateManager, Neo_Single m_single_neo){
        super(stateManager,m_single_neo);
    }

    @Override
    public String Current_State(){
        return "Single state";
    }

    @Override
    public Command OnSwitchCommand(){return Commands.none();}


    @Override
    public Command Button_1(){
       //Check that command is valid if not returns Commands.none() 
       return CommandHelper.returnIfValid(()->new One(m_single_neo));

    }

    @Override
    public Command Button_2(){
        return Commands.none();
    }

    @Override
    public Command Button_3(){
        return Commands.none();
    }

    @Override
    public Command Button_4(){
        return Commands.none();
    }

    @Override
    public Command Button_5(){
        return Commands.none();
    }

    @Override
    public Command Button_6(){
        return Commands.none();
    }

    @Override
    public Command Button_7(){
        return Commands.none();
    }

    @Override
    public Command Button_8(){
        return Commands.none();
    }

    @Override
    public Command Button_9(){
        return Commands.none();
    }

    @Override
    public Command Button_10(){
        return Commands.none();
    }

    @Override
    public Command Button_11(){
        return Commands.none();
    }

    @Override
    public Command Button_12(){
        return Commands.none();
    }


}