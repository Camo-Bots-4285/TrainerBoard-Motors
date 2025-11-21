package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.util.LoggerHelper;

import frc.robot.Constants.SingleMotorConstants;
import frc.robot.Constants.SingleMotorConstants.Setpoint;


public class SingleMotor extends SubsystemBase {

    private final FlywheelMechanism io;

    public SingleMotor(FlywheelMechanism io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(SingleMotorConstants.NAME, this);
        io.periodic();
    }
       
    public Command setVelocity(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runVelocity(SingleMotorConstants.CRUISE_VELOCITY,
                SingleMotorConstants.ACCELERATION, PIDSlot.SLOT_1))
            .withName("Go To "  + setpoint.name() + " Setpoint");
    };

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }

}
