// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HumanInterface.ElasticDisplay;
import frc.robot.lib.Logger.AdvantageKitLogger;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
 public static double Time;
 public double autoStartTime;
 public double autoTime;
 public double teleOpStartTime;
 public double teleOpTime;
 private  RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();


//This will build and auto in the start to make pathplanner run faster
// DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
PathfindingCommand.warmupCommand().schedule();

    AdvantageKitLogger.initialize(this);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    AdvantageKitLogger.periodicUpdate();

    Time = Timer.getFPGATimestamp();

  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    autoStartTime=Time;

    Logger.recordOutput("Robot/In_Use", ElasticDisplay.mChooser_Robot.getSelected());
    Logger.recordOutput("Robot/Battery", ElasticDisplay.mChooser_Battery.getSelected());
    Logger.recordOutput("Robot/Auto", ElasticDisplay.mChooser_Auto.getSelected());
  }

  @Override
  public void autonomousPeriodic() {
    autoTime = Time - autoStartTime+1;

    //  m_robotContainer.m_aprilTag.updatedPoseFromTagAuto();
  

    autoTime = Math.abs(15-autoTime);
    Logger.recordOutput("Robot/Time", autoTime);
    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    teleOpStartTime = Time;

    
    //Initail commands
    m_robotContainer.scheduleInitialCommands();


    
  }

  @Override
  public void teleopPeriodic() {
    teleOpTime = Time - teleOpStartTime+1;

    Logger.recordOutput("Robot/Time", teleOpTime);  

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
