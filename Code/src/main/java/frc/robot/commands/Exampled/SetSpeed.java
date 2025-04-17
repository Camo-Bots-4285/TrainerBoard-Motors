// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Exampled;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CTRE_Subsytem;
import frc.robot.subsystems.Neo_Single;

import java.util.function.BiFunction;


/** An example command that uses an example subsystem. */
public class SetSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Neo_Single m_REVSubsytem;

  public SetSpeed(Neo_Single subsystem) {
    m_REVSubsytem = subsystem;
      addRequirements(subsystem);
    }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_REVSubsytem.Leader.setMotorPercent(1);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_REVSubsytem.Leader.setMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

        return false;  

    }   
}