// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.SingleMotor;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.FlyWheelSubsystem;


// /** An example command that uses an example subsystem. */
// public class SingleSetSpeed extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//   private FlyWheelSubsystem m_singleSubsytem;


//   public SingleSetSpeed(FlyWheelSubsystem m_singleSubsytem) {
//     this.m_singleSubsytem=m_singleSubsytem;
//       addRequirements(m_singleSubsytem);
//     }
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     m_singleSubsytem.FlyWheel_Motor.setTargetMotorVelocity(4000);

// }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_singleSubsytem.FlyWheel_Motor.setTargetMotorVelocity(0);

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {

//         return false;  

//     }   
// }