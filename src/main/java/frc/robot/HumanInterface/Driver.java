package frc.robot.HumanInterface;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.HumanInterface.StateMachine.StateManager;
import frc.robot.HumanInterface.StateMachine.State_Double;
import frc.robot.HumanInterface.StateMachine.State_Single;
import frc.robot.HumanInterface.StateMachine.StateManager.BindType;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

/**
 * DriverInterface
 * <p>
 * Holds all buttons mapped to the driver joystick.
 * <p>
 * TODO: Add more buttons and map commands as needed.
 */
public class Driver {

    //Defines driver joystick
    private final static Joystick driverJoystick = new Joystick(0);
    
        private StateManager stateManager;
        private FlyWheelSubsystem m_flyWheel;
        private ClimberSubsystem m_climber;
    
        /**
         * Constructor
         * 
         * @param stateManager state manager instance
         * @param m_flyWheel Neo_Single subsystem
         * @param drivetrain Swerve drivetrain subsystem
         */
        public Driver(StateManager stateManager, FlyWheelSubsystem m_flyWheel, ClimberSubsystem m_climber) {
            this.stateManager = stateManager;
            this.m_flyWheel = m_flyWheel;
            this.m_climber = m_climber;
        }
    
    
        // Define buttons
    private JoystickButton btn_1 = new JoystickButton(driverJoystick, 1);
    private JoystickButton btn_2 = new JoystickButton(driverJoystick, 2);
    private JoystickButton btn_3 = new JoystickButton(driverJoystick, 3);
    private JoystickButton btn_4 = new JoystickButton(driverJoystick, 4);
    private JoystickButton btn_5 = new JoystickButton(driverJoystick, 5);
    private JoystickButton btn_6 = new JoystickButton(driverJoystick, 6);
    private JoystickButton btn_7 = new JoystickButton(driverJoystick, 7);
    private JoystickButton btn_8 = new JoystickButton(driverJoystick, 8);
    private JoystickButton btn_9 = new JoystickButton(driverJoystick, 9);
    private JoystickButton btn_10 = new JoystickButton(driverJoystick, 10);
    private JoystickButton btn_11 = new JoystickButton(driverJoystick, 11);
    private JoystickButton btn_12 = new JoystickButton(driverJoystick, 12);


    public JoystickButton[] buttonArray = {btn_1,btn_2,btn_3,btn_4,null,null,btn_7,btn_8,btn_9,btn_10,btn_11,btn_12};
    public  BindType[] bindTypeArray ={
        BindType.WHILE_TRUE, BindType.WHILE_TRUE, BindType.TOGGLE_ON_TRUE,BindType.WHILE_TRUE,BindType.WHILE_TRUE,
        BindType.WHILE_TRUE,BindType.WHILE_TRUE,BindType.WHILE_TRUE,BindType.WHILE_TRUE,BindType.WHILE_TRUE,
        BindType.WHILE_TRUE,BindType.WHILE_TRUE 
    };


        /**
         * Sets up all button bindings.
         */
        public void DriverBindings() {
            //StateManagerSetup();
            // btn_5.whileTrue(new Arm45Degrees(m_climber));
            // btn_6.whileTrue(new Arm0Degrees(m_climber));

            // btn_4.whileTrue(new TimedMovements(m_climber));

            // m_climber.setDefaultCommand(new ArmMovementJoy(m_climber, driverJoystick));


            

        }

    /**
     * Sets initial state, button bindings, and binds buttons that switch states.
     */
    public void StateManagerSetup() {
        //Defines the instance very import to keep the same because state check looks for instance not class
        State_Double doubleState = new State_Double(stateManager, m_flyWheel, m_climber);
        State_Single singleState = new State_Single(stateManager, m_flyWheel, m_climber);
        
        //Map button for doubleState
        stateManager.bindAllButtonsForState(
            doubleState,
            buttonArray,
            bindTypeArray
        );  
        //Map button for singleState
        stateManager.bindAllButtonsForState(
            singleState,
            buttonArray,
            bindTypeArray
        );   

        // Start in State_Double by default
        stateManager.setState(doubleState);

        // Switch to State_Double on button 5 press
        btn_5.onTrue(new InstantCommand(() -> {
            stateManager.setState(doubleState);
        }));

        // Switch to State_Single on button 6 press
        btn_6.onTrue(new InstantCommand(() -> {
            stateManager.setState(singleState);
        }));
    }
}
