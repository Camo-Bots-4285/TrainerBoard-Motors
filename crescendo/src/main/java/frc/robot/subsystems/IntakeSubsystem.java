// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAlternateEncoder;
// import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IntakeConstants;


// public class IntakeSubsystem extends SubsystemBase {
//   /** Intake Subsystem - Feeds Notes */

//   private CANSparkMax floor_feeder_motor;
//   private SparkPIDController floor_feeder_pid;
//   private RelativeEncoder floor_feeder_encoder;
  


//   private CANSparkMax floor_feeder_motor2;
//   private SparkPIDController floor_feeder_pid2;
//   private RelativeEncoder floor_feeder_encoder2;

//   public static PIDController PIDWheel = new PIDController(0.285, 0, 0.0);//0.0025, .285
//   private double floor_feeder_motor_speed;
//   private double Speed1;
//   public IntakeSubsystem() {

//     floor_feeder_motor = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_FLOOR, MotorType.kBrushless);
//     floor_feeder_encoder = floor_feeder_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

//     floor_feeder_motor_speed = floor_feeder_encoder.getVelocity();  
//     floor_feeder_motor_speed = floor_feeder_encoder.getPosition();
//    // floor_feeder_motor_speed_inmeterpersecond = floor_feeder_encoder.getVelocity()*0.1;
//     Speed1 = PIDWheel.calculate(1.0,floor_feeder_motor_speed);

    
//     floor_feeder_motor2 = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_FLOOR2, MotorType.kBrushless);
//     floor_feeder_encoder2 = floor_feeder_motor2.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
//     floor_feeder_pid2 = floor_feeder_motor2.getPIDController();
//     floor_feeder_pid2.setFeedbackDevice(floor_feeder_encoder2); 

//   }
  

//   /**
//    * @param speed
//    * @param direction
//    */
//   public void feed(double speed, boolean direction) {
//     if (direction) {
  

//       floor_feeder_motor.set(PIDWheel.calculate(speed,floor_feeder_motor_speed));
//       floor_feeder_motor2.set(speed);
//     }
//     else {
//       floor_feeder_motor.set(-speed);
//       floor_feeder_motor2.set(-speed);
//     }
//   }

//   public void stop() {
//     floor_feeder_motor.set(0.0);
//     floor_feeder_motor2.set(0.0);
//   }

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     System.out.println();
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
