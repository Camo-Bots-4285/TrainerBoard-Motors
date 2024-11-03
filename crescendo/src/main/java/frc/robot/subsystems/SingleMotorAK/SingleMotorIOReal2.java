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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SingleMotorConstants;

/*this is for a talan fx controlr falcon/talon*/
public class SingleMotorIOReal2 implements SingleMotorIO {
  

  private final TalonFX SingleMotor = new TalonFX(SingleMotorConstants.SINGLE_MOTOR);

  private final StatusSignal<Double> SingleMotorPosition = SingleMotor.getPosition();
  private final StatusSignal<Double> SingleMotorVelocity = SingleMotor.getVelocity();
  private final StatusSignal<Double> SingleMotorAppliedVolts = SingleMotor.getMotorVoltage();
  private final StatusSignal<Double> SingleMotorCurrent = SingleMotor.getSupplyCurrent();


  public SingleMotorIOReal2() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SingleMotorConstants.MAX_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    SingleMotor.getConfigurator().apply(config);


    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, SingleMotorPosition, SingleMotorVelocity, SingleMotorAppliedVolts, SingleMotorCurrent);
    SingleMotor.optimizeBusUtilization();

  }

  @Override
  public void updateInputs(SingleMotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        SingleMotorPosition, SingleMotorVelocity, SingleMotorAppliedVolts, SingleMotorCurrent);
    inputs.positionRad = Units.rotationsToRadians(SingleMotorPosition.getValueAsDouble()) / SingleMotorConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(SingleMotorVelocity.getValueAsDouble()) / SingleMotorConstants.GEAR_RATIO;
    inputs.appliedVolts = SingleMotorAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {SingleMotorCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    SingleMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    SingleMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    SingleMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    SingleMotor.getConfigurator().apply(config);
  }
}