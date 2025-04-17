package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.REV_Single;
import frc.robot.util.Rev_Hardware.REV_Follower;
import frc.robot.util.Rev_Hardware.REV_Max;


public class Neo_Single extends SubsystemBase { 



  public Neo_Single() {}

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor CAN ID
   * drive motor PID value P
   * rotation motor can ID
   * rotation motor PID value P
   * external CANCoder can ID
   * measured CANCoder offset
   */

   public final static REV_Max Leader = new REV_Max(
    REV_Single.MotorIdentification,
    REV_Single.Motor_int,
    REV_Single.Motor_double,
    REV_Single.Motor_boolean,
    REV_Single.TunnerValues,
    REV_Single.MotionConstants);



  @Override
  public void periodic() {
   
    LiveFeed();
  }

  public void LiveFeed(){
    SmartDashboard.putNumber("Single_Neo/Motor_Position", Leader.getMotorPosition().getRotations());
    SmartDashboard.putNumber("Single_Neo/Mechanical_Position", Leader.getMechanismPosition().getRotations());
    SmartDashboard.putNumber("Single_Neo/Motor_Velocity", Leader.getMotorVelocity().getRotations());
    SmartDashboard.putNumber("Single_Neo/Mechanical_Velocity", Leader.getMechanismVelocity().getRotations());
    SmartDashboard.putNumber("Single_Neo/Amps", Leader.getAmps());
  }

  }

