package frc.robot.control;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.SingleMotor.One;
import frc.robot.control.StateMachine.State.*;
import frc.robot.control.StateMachine.StateManager;
import frc.robot.control.StateMachine.State_Double;
import frc.robot.control.StateMachine.State_Single;
import frc.robot.lib.CommandHelper;
import frc.robot.subsystems.Neo_Single;

/**
 * DriverInterface
 * <p>
 * Description:
 * This class will hold all button that should be mapped to the driverjoystick 
 * <p>
 * Notes: This will change from year to year
 * <p>
 * TODO: Add more button and map them to commands 
 */
public class DriverInterface {

  private final Joystick driverJoystick = new Joystick(0);

  private StateManager stateManager;
  private Neo_Single m_single_neo;

    /**
     * DriverInterface - Constructor
     * <p>
     * Description:
     * Passes thought subsystem to be pass thought to commands
     */
  public DriverInterface( StateManager stateManager,Neo_Single m_single_neo){
    this.stateManager = stateManager;
    this.m_single_neo=m_single_neo;
  }

    /**
     * DriverInterface - Bindings
     * <p>
     * Description:
     * Holds the button mapping for the DriverInterface to be placed in RobotContainer for initation
     */
    public void DriverBindings(){
      State_Manger();
    }

    /**Shows how to hold a group of command that can be use to call multiple commands on state switch */
    Command stateCommands = new SequentialCommandGroup(
      stateManager.getState().Button_1()
    );

    public void State_Manger(){

      // Inside your configureBindings() method or constructor
      JoystickButton modeSwitchButton = new JoystickButton(driverJoystick, 5); // Button 2
      modeSwitchButton.onTrue(new InstantCommand(() -> 
          stateManager.setState(new State_Double(stateManager,m_single_neo)) // Switch to a new state
      ));

      JoystickButton modeSwitchButton1 = new JoystickButton(driverJoystick, 6);
      modeSwitchButton1.onTrue(new InstantCommand(() -> {
        stateManager.setState(new State_Single(stateManager,m_single_neo)); // Switch to a new state
        stateCommands.cancel();
      }));

      //Calls the Button one command
        JoystickButton button1 = new JoystickButton(driverJoystick, 1);
        button1.onTrue(stateManager.getState().Button_1());//Does not need to check command because command was checked in statemachine
    }                                                                                      
    
}
