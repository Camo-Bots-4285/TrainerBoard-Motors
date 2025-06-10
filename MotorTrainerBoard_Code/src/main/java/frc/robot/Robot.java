// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BuildConstants;
import frc.robot.control.ElasticInterface;

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


    //Advantage kit logger 

    //Add Infromation to help identify logs
    Logger.recordMetadata("Code_ProjectName", BuildConstants.PROJECT_NAME);
    Logger.recordMetadata("Code_Version", BuildConstants.VERSION);
    Logger.recordMetadata("Code_BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Git_Version_Changed", BuildConstants.GIT_REVISION);
    Logger.recordMetadata("Git_Branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Robot/Constants", Constants.config.ConstantsName());


    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    //LoggedPowerDistribution.getInstance(50, ModuleType.kRev); // Example: PDH on CAN ID 50
} else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

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

    Logger.recordOutput("Robot/In_Use", ElasticInterface.mChooser_Robot.getSelected());
    Logger.recordOutput("Robot/Battery", ElasticInterface.mChooser_Battery.getSelected());
    Logger.recordOutput("Robot/Auto", ElasticInterface.mChooser_Auto.getSelected());
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
