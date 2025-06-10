package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Falcon_Motor;
import frc.robot.configs.MotorConstants;
import frc.robot.lib.CTRE_Hardware.*;



public class CTRE_Subsytem extends SubsystemBase {
 
    public CTRE_Subsytem(){}

    public final CTRE_Talon Leader = new CTRE_Talon(
        Falcon_Motor.MotorIdentification,
        Falcon_Motor.CANLoop,
        MotorConstants.Falcon_int(Falcon_Motor.Motor_ID),
        MotorConstants.Falcon_double(Falcon_Motor.Gear_Ratio,Falcon_Motor.Wheel_Radius),
        Falcon_Motor.Idle_Mode,
        Falcon_Motor.Inverted,
        Falcon_Motor.MotionProfile
     );

     // public final CTRE_Follower Follower = new CTRE_Follower(
     //    "Follower_bob", 
     //    2,
     //    "rio",
     //    Leader.getMotorID(),
     //    Leader.get_Follower_Motor_doubles(),
     //    true
     // );

         @Override
    public void periodic() {
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Motor_Position", Leader.getMotorPosition().getDegrees());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Motor_Velocity", Leader.getMotorVelocity_RPM().getDegrees());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Mech_Position", Leader.getMechanismPosition().getDegrees());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Mech_Velocity", Leader.getMechanismVelocity_RPM().getDegrees());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Amps", Leader.getAmps());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Volts", Leader.getVoltage());
     Logger.recordOutput(Falcon_Motor.MotorIdentification+"/Temp", Leader.getTemputerInCelcius());

     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Motor_Position", Follower.getMotorPosition().getDegrees());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Motor_Velocity", Follower.getMotorVelocity().getDegrees());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Mech_Position", Follower.getMechanismPosition().getDegrees());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Mech_Velocity", Follower.getMechanismVelocity().getDegrees());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Amps", Follower.getAmps());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Volts", Follower.getVoltage());
     // Logger.recordOutput(Falcon_Motor.MotorIdentification_Follower+"/Temp", Follower.getTemputerInCelcius());
    }

}
