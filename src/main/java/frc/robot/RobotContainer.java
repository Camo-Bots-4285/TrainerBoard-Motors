// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
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
                m_singleMotor = new SingleMotor(SingleMotorConstants.getReal());
                m_doubleMotor = new DoubleMotor(DoubleMotorConstants.getReal());
            }

            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                m_singleMotor = new SingleMotor(SingleMotorConstants.getSim());
                m_doubleMotor = new DoubleMotor(DoubleMotorConstants.getSim());
            }

            default -> {
                // Replayed robot, disable IO implementations
                m_singleMotor = new SingleMotor(SingleMotorConstants.getReplay());
                m_doubleMotor = new DoubleMotor(DoubleMotorConstants.getReplay());
            }
        }

        // Configure Bindings from HumanInterface 
        // m_DriverInterface.DriverBindings();
        // m_coDriver.CoDriverBindings();

        m_singleMotor.setDefaultCommand(m_singleMotor.setVelocity(SingleMotorConstants.Setpoint.INTAKE));
        m_doubleMotor.setDefaultCommand(getAutonomousCommand());
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
