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

package frc.robot.subsystems.SingleMotorAK;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants.SingleMotorConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class SingleMotorReal implements SingleMotorIO {


  private final CANSparkMax SingleMotor = new CANSparkMax(SingleMotorConstants.SINGLE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder encoder = SingleMotor.getEncoder();
  private final SparkPIDController pid = SingleMotor.getPIDController();



  // private final CANSparkMax SingleMotor = new CANSparkMax(0, MotorType.kBrushless);
  // private final CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);
  // private final RelativeEncoder encoder = SingleMotor.getEncoder();
  // private final SparkPIDController pid = SingleMotor.getPIDController();

  public SingleMotorReal() {
    
    SingleMotor.restoreFactoryDefaults();
    SingleMotor.setCANTimeout(250);
    SingleMotor.setInverted(false);
    SingleMotor.enableVoltageCompensation(SingleMotorConstants.VOLTAGE_COMP_SINGLE_MOTOR);
    SingleMotor.setSmartCurrentLimit(SingleMotorConstants.MAX_AMPS);
    SingleMotor.burnFlash();
 
  }

  @Override
  public void updateInputs(SingleMotorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / SingleMotorConstants.GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / SingleMotorConstants.GEAR_RATIO);
    inputs.appliedVolts = SingleMotor.getAppliedOutput() * SingleMotor.getBusVoltage();
    inputs.currentAmps = new double[] {SingleMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    SingleMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * SingleMotorConstants.GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    SingleMotor.stopMotor();
  }


  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}