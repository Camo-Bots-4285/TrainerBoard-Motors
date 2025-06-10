// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Exampled;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CTRE_Subsytem;
import frc.robot.subsystems.Neo_Double;
import frc.robot.subsystems.Neo_Single;

import java.util.function.BiFunction;


/** An example command that uses an example subsystem. */
public class SetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Neo_Single m_singleSubsytem;
  private Neo_Double m_doubleSubsytem;

  public SetSpeed(Neo_Single m_singleSubsytem,Neo_Double m_doubleSubsytem) {
    this.m_singleSubsytem=m_singleSubsytem;
    this.m_doubleSubsytem=m_doubleSubsytem;
      addRequirements(m_singleSubsytem,m_doubleSubsytem);
    }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_singleSubsytem.Single_Motor.setMotorPercent(0.5);
    m_doubleSubsytem.Leader.setMotorPercent(0.1);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_singleSubsytem.Single_Motor.setMotorStop();
    m_doubleSubsytem.Leader.setMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

        return false;  

    }   
}