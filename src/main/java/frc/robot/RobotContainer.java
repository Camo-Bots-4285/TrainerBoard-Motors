// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DoubleMotorConstants;
import frc.robot.Constants.SingleMotorConstants;
import frc.robot.HumanInterface.CoDriver;
import frc.robot.HumanInterface.Driver;
import frc.robot.HumanInterface.ElasticDisplay;
import frc.robot.HumanInterface.StateMachine.StateManager;
import frc.robot.subsystems.DoubleMotor;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.SingleMotor;



@SuppressWarnings("unused")
/**
 * The RobotContainer class is the central hub for defining the robot's subsystems,
 * commands, and button mappings.
 *
 * <p>This class handles the instantiation and configuration of all subsystems,
 * operator interfaces, and autonomous command routines. It also manages
 * scheduling of initial commands when TeleOp begins.
 *
 * <p>Subsystems such as the drivetrain, vision, and control interfaces are created here,
 * and commands are provided for autonomous and teleoperated operation.
 *
 * <p>All button bindings and command scheduling should be configured within this class.
 *
 */
public class RobotContainer {

    private final SingleMotor m_singleMotor;
    private final DoubleMotor m_doubleMotor;

    //private final ArmSubsytem m_FlyWheel = new ArmSubsytem();
    //private final Motors m_motor = new Motors();

    // Intatating subsytems
    // private final FlyWheelSubsystem m_flyWheel = new FlyWheelSubsystem();
    // private final ClimberSubsystem m_climber = new ClimberSubsystem();

    // Intatating Control Interface Make sure to pass thought the subsytem dependecies
    // private final StateManager m_stateManger = new StateManager();
    // private final CoDriver m_coDriver = new CoDriver(m_flyWheel);
    // private final Driver m_DriverInterface = new Driver(m_stateManger, m_flyWheel, m_climber);
    // private final ElasticDisplay m_ElasticInterface = new ElasticDisplay();
    // private final Tunning m_TunningInterface = new Tunning(m_flyWheel);

    Joystick driverJoystick = new Joystick(0);

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

    
    /**
     * Constructs a new RobotContainer.
     *
     * <p>Initializes all subsystems, operator interfaces, and telemetry.
     * Configures button bindings for driver and co-driver controls.
     * Also registers telemetry logging for the drivetrain subsystem.
     */
    public RobotContainer() { 
        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                m_singleMotor = SingleMotorConstants.getReal();
                m_doubleMotor = DoubleMotorConstants.getReal();
            }

            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                m_singleMotor = SingleMotorConstants.getSim();
                m_doubleMotor = DoubleMotorConstants.getSim();
            }

            default -> {
                // Replayed robot, disable IO implementations
                m_singleMotor = SingleMotorConstants.getReplay();
                m_doubleMotor = DoubleMotorConstants.getReplay();
            }
        }

        //m_singleMotor.setDefaultCommand(m_singleMotor.setSetpoint(Rotation.of(1)));
        
        
        m_singleMotor.setDefaultCommand(m_singleMotor.setVelocity(SingleMotorConstants.Setpoint.STOP));
        m_doubleMotor.setDefaultCommand(m_doubleMotor.setSetpoint(DoubleMotorConstants.DEFAULT_SETPOINT));

        btn_4.whileTrue(m_singleMotor.setVelocity(SingleMotorConstants.Setpoint.INTAKE).alongWith(m_doubleMotor.setSetpoint(DoubleMotorConstants.Setpoint.RAISED)).withName("Intake"));
        btn_3.whileTrue(m_singleMotor.setVelocity(SingleMotorConstants.Setpoint.UNJAM).alongWith(m_doubleMotor.setSetpoint(DoubleMotorConstants.Setpoint.RAISED)).withName("Unjam"));


    }

    /**
     * Schedules commands to be run at the start of TeleOperated mode.
     *
     * <p>Typically includes initializing commands for operator controls
     * and any other commands that need to start when TeleOp begins.
     */
    public void scheduleInitialCommands() {
        /* IntCommands from HumanInterface */
      // m_coDriver.CoDriverIntCommands();

    }    
    
    //TODO: Should we make a class to house auto stuff?

    /**
     * Defines and registers autonomous commands available for use during the autonomous period.
     *
     * <p>This method can be used to map command names to specific autonomous actions,
     * allowing easy selection and scheduling of autonomous routines.
     */
    public void autoCommands() {
        // Map<String, Command> namedCommands = Map.ofEntries(
        // Map.entry("L1", new RunCommand(() -> m_arm.setPositionByIndex(5))),
        // Map.entry("L1Pre", new RunCommand(() -> m_arm.setPositionByIndex(4))),
        // Map.entry("BallDrop", m_thrower.pivotUnhook()),
        // Map.entry("StopDrive", new StopDrive(m_swerveBase)),
        // Map.entry("Eject", new RunCommand(() -> EjectesGamePiece = true)),
        // Map.entry("EjectStop", new RunCommand(() -> EjectesGamePiece = false))
        // );

        // // Register all at once
        // namedCommands.forEach(NamedCommands::registerCommand);
    }

    /**
     * Builds and returns the autonomous command to run during the autonomous period.
     *
     * <p>This method constructs the selected autonomous routine, which will be
     * scheduled when autonomous mode begins.
     *
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Test");
    }

}
