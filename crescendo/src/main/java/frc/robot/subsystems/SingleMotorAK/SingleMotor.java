// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.



/*The following code is for a Neo run off of a spark max
 * This code will spin a the wheel at a set tip speed and
 * Move the whel to a know location using the shotest distance
 */
package frc.robot.subsystems.SingleMotorAK;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SingleMotorConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

  public class SingleMotor extends SubsystemBase {
  public static int SINGLE_MOTOR;
  private final SingleMotorIO singleMotorIO;
  private final SingleMotorIOInputsAutoLogged inputs = new SingleMotorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  public static double positionDeg;
  public static double RPM;


  /** Creates a new SingleMotor. */
  public SingleMotor(SingleMotorIO io) {
   this.singleMotorIO = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        singleMotorIO.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        singleMotorIO.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SingleMotor/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    /*Standerd Advantage Kit Code to log */
    singleMotorIO.updateInputs(inputs);
    Logger.processInputs("SingleMotor", inputs);
    
    /*Log SingleMotor setpoint*/
    Logger.recordOutput("SingleMotor/Position0-360", getPosition0_360());

    positionDeg = SmartDashboard.getNumber("SetDergee", 0);
    RPM = SmartDashboard.getNumber("SetRPM", 0);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    singleMotorIO.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    singleMotorIO.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log SingleMotor setpoint
    Logger.recordOutput("SingleMotor/SetpointRPM", velocityRPM);
  }

  /** Stops the SingleMotor. */
  public void stop() {
    singleMotorIO.stop();
  }

  /*Gets the postion of the wheel and wraps it form 0 - 2PI */
  public double getPosition0_360() {
    /*Divide the motor reading and out puts the remander */
    double wheelAngle = inputs.positionRad % (2*Math.PI);
  
    /*Adds 2PI to make reading from 0 - 2PI*/
      if (wheelAngle < 0.0){
        wheelAngle += 2.0 * Math.PI;
        }
    /*Return the wrapped angle in radians*/
      double wrappedAngle = wheelAngle;
      return wrappedAngle;

   
  }

    /* Sets position from 0-360 will take the shortest path */
  public void setPosition(double positionDeg ) {
    /*Sets Varibale that will only be used in thsi method*/
    double PosDiff = positionDeg - getPosition0_360();
    double PosWarp;

    /*Uses wrapped angle and desired postion to fins the shortest path*/
    if(PosDiff<-180){
       PosWarp = PosDiff + 360;
    }
   else{PosWarp = PosDiff + 360;}

   /*Set the differnce beteen desired state and current to zero and moves there using the error 
    * Note there is more advance way of moveing and mechanisum
   */
  double setvoltage = SingleMotorConstants.PID_Positon_Single_Motor.calculate(PosWarp,0);
    singleMotorIO.setVoltage(setvoltage);

    /*Logs Values to help problme solve if something oes wrong */
    Logger.recordOutput("SingleMotor/PositionSetOffset", positionDeg);
    Logger.recordOutput("SingleMotor/PositionSetWrap", PosWarp);
    Logger.recordOutput("SingleMotor/PositonSetVoltage", PosWarp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}