package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.util.LoggerHelper;
import frc.robot.Constants.DoubleMotorConstants;
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
        Logger.recordOutput("3D_Feild/0_Zero", new Pose3d[] {new Pose3d(0,0,0, new Rotation3d(0, 0, -0))});
        Logger.recordOutput("3D_Feild/1_SingleWheel", new Pose3d[] {new Pose3d(0.04445,-0.127,0.6223, new Rotation3d(0, 0, io.getPosition().in(Radians)))});
        
    }
       
    public Command setVelocity(Setpoint setpoint)
    {

        return this.run(
            () -> io.runVelocity(setpoint.getSetpoint(),
                SingleMotorConstants.ACCELERATION, PIDSlot.SLOT_1))
            .withName("Go To "  + setpoint.name() + " Setpoint - "+setpoint.getSetpoint().in(RotationsPerSecond));
            
    }; 
    public Command setSetpoint(Angle postion)
    {
        return this.runOnce(
            () -> io.runPosition(postion,
                DoubleMotorConstants.CRUISE_VELOCITY,
                DoubleMotorConstants.ACCELERATION, DoubleMotorConstants.JERK,
                PIDSlot.SLOT_0))
            .withName("Go To "  + postion.in(Rotation) + " Setpoint");
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
