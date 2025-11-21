// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Exampled;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.FlyWheelSubsystem;


// /** An example command that uses an example subsystem. */
// public class SetSpeed extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//   private FlyWheelSubsystem m_singleSubsytem;
//   private ClimberSubsystem m_doubleSubsytem;

//   public SetSpeed(FlyWheelSubsystem m_singleSubsytem,ClimberSubsystem m_doubleSubsytem) {
//     this.m_singleSubsytem=m_singleSubsytem;
//     this.m_doubleSubsytem=m_doubleSubsytem;
//       addRequirements(m_singleSubsytem,m_doubleSubsytem);
//     }
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
  
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     m_singleSubsytem.FlyWheel_Motor.setPercent(0.5);
//     m_doubleSubsytem.leaderMotor.setPercent(0.1);
// }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_singleSubsytem.FlyWheel_Motor.stop();
//     m_doubleSubsytem.leaderMotor.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {

//         return false;  

//     }   
// }