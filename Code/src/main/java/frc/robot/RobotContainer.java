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

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Exampled.*;
import frc.robot.subsystems.Neo_Single;
import frc.robot.util.Cameras.DriveCamera;
import frc.robot.util.Rev_Hardware.REV_Max;


public class RobotContainer {

    private final Joystick driverJoystick = new Joystick(0);;

    public static Neo_Single m_revSubsytem = new Neo_Single();

    private GenericEntry[] buttons;

    public RobotContainer() {

        ShuffleboardTab tab = Shuffleboard.getTab("Teleoperated");
        buttons = new GenericEntry[]{
            tab.add("RunMotor", false).withWidget("Toggle Button").getEntry(),
        };
        

        Trigger fireTrigger = getDashboardEntryAsTrigger(buttons[0]);
        
        fireTrigger.whileTrue(new SetSpeed(m_revSubsytem));
        //fireTrigger.whileFalse(new Zero(m_revSubsytem));
       

    // JoystickButton btn_run_motor = new JoystickButton(driverJoystick, 1);
    //   btn_run_motor.onTrue(new SetSpeed());

    

    }
    
    public static DriveCamera main = new DriveCamera(
        0,
        60,
        480,
        320
    );

    // public static DriveCamera second = new DriveCamera(
    //     1,
    //     60,
    //     480,
    //     320
    // );

    public Trigger getDashboardEntryAsTrigger(GenericEntry dashboardEntry){
        return new Trigger(()-> dashboardEntry.getBoolean(false));
      }
   


    public Command getAutonomousCommand() {
            return AutoBuilder.buildAuto("Test");
    }
}
