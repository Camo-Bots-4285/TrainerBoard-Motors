// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TODO
 * 
 * Make sure all values are up to date not sure if this is an example of has already been run though out system
 * 
 * Thing to test
 * TeleOP Driving on x_box first learn what each button does and comment that. This way I can use them.
 * Robot on block is a good idea here.
 * 
 * after that test CTRE  and TeleOpSwerve and notice the differnce in contorl 
 * 
 * If robot is working as intended run camera method in robot period and see if odemetyr takes
 * 
 * If no camera or they work. Then let move to running autos. Create the auto that stated center and moved to each side as a base line.
 * Test PID to make sure they as good as they can be. then run center with each side and see how much more accrate 
 * then WPILib swerve was without cameras. then add cameras carefull if PID are wrong this will not be fun.
 * 
 * Please repost you finding to me anytime in this process. When Driving works thougt TeleOp swerve and you give it back to me with any 
 * changes and I will empliment selfdriving with Util class to make it easier to use and less math focused.
 * 
 * After that questNav will be introduct the code. I will keep an eye on the code beacsue it is changeing rapidly and new code 
 * that allows quest ot see tags drops  after champs which can me a big game changer and change the way we code it 
 * because it will not need photon ;)
 */

package frc.robot;


import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.REV_Double;
import frc.robot.Constants.REV_Single;
import frc.robot.commands.Exampled.*;
import frc.robot.commands.SingleMotor.One;
import frc.robot.commands.SingleMotor.SingleSetSpeed;
import frc.robot.commands.SingleMotor.Zero;
import frc.robot.control.CoDriverInterface;
import frc.robot.control.DriverInterface;
import frc.robot.control.ElasticInterface;
import frc.robot.control.TunningInterface;
import frc.robot.control.StateMachine.StateManager;
import frc.robot.lib.Cameras.DriveCamera;
import frc.robot.lib.Communication.ButtonGroup;
import frc.robot.lib.Communication.ButtonSingle;
import frc.robot.lib.Rev_Hardware.REV_Max;
import frc.robot.subsystems.Neo_Double;
import frc.robot.subsystems.Neo_Single;



public class RobotContainer {

    //Intatating subsytems
    //The following will set the subsystem if the value is true if not it is going to set it to null
    public final Neo_Single m_single_neo = REV_Single.ENABLE_SINGLE_NEO ? new Neo_Single() : null;
    public final Neo_Double m_double_neo = REV_Double.ENABLE_DOUBLE_NEO ? new Neo_Double() : null;


    //Intatating Control Interface Make sure to pass thought the subsytem dependecies
    public  StateManager m_stateManger = new StateManager(m_single_neo);
    public  DriverInterface m_DriverInterface = new DriverInterface(m_stateManger,m_single_neo);
    public  CoDriverInterface m_coDriver = new CoDriverInterface(m_single_neo);
    public  ElasticInterface m_ElasticInterface = new ElasticInterface();
    public  TunningInterface m_TunningInterface = new TunningInterface(m_single_neo); 

    //CO: This allows command to start when turned on turned of for saftey
    // private final Command m_spinCommand;
    // private final Trigger btn_Spin_All;

    public RobotContainer() {
        

        m_DriverInterface.DriverBindings();
       //Example of a Digital Button that start when robot turn on 
    //CO: This allows command to start when turned on turned of for saftey
    //    m_spinCommand = new SetSpeed(m_single_neo, m_double_neo);
    //    btn_Spin_All = b_Spin.getDashboardEntryAsTrigger();
    //    // Bind it to run while true
    //    btn_Spin_All.whileTrue(m_spinCommand);


        // Trigger btn_single_zero = ButtonSingle.getDashboardEntryAsTrigger(ButtonSingle.createButton( table, "Single-Zero", false),false);
        // btn_single_zero.whileTrue(new Zero(m_single_neo));


        // //Example of Using a Physical Button on a controler
        // //  JoystickButton btn_run_motor = new JoystickButton(driverJoystick, 1);
        // //  btn_run_motor.whileTrue(new SetSpeed(m_single_neo,m_double_neo));




    }

public void autoCommands(){
    //  Map<String, Command> namedCommands = Map.ofEntries(
    //     Map.entry("L1", new RunCommand(() -> m_arm.setPositionByIndex(5))),
    //     Map.entry("L1Pre", new RunCommand(() -> m_arm.setPositionByIndex(4))),
    //     Map.entry("BallDrop", m_thrower.pivotUnhook()),
    //     Map.entry("StopDrive", new StopDrive(m_swerveBase)),
    //     Map.entry("Eject", new RunCommand(() -> EjectesGamePiece = true)),
    //     Map.entry("EjectStop", new RunCommand(() -> EjectesGamePiece = false))
    // );

    // // Register all at once
    // namedCommands.forEach(NamedCommands::registerCommand);
}
    
    public static DriveCamera main = new DriveCamera(
        0,
        60,
        480,
        320
    );



    public Command getAutonomousCommand() {
            return AutoBuilder.buildAuto("Test");
    }

    /**
     * Holds command that arre to be schedule in the start of TeleOp
     */
    public void scheduleInitialCommands() {
    //CO: This allows command to start when turned on turned of for saftey
        // if (btn_Spin_All.getAsBoolean()) {
        //     m_spinCommand.schedule();
        // }
    }

}
