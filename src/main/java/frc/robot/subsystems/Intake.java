package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.util.LoggerHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeOptions;
import frc.robot.Constants.SingleMotorConstants;
import frc.robot.Constants.DoubleMotorConstants;




public class Intake extends SubsystemBase{

    private final FlywheelMechanism Wheels;
    private final FlywheelMechanism Pivot;

    public Intake(FlywheelMechanism Wheels,FlywheelMechanism Pivot)
    {
        this.Wheels = Wheels;
        this.Pivot = Pivot;
    }

@Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(IntakeConstants.NAME, this);
        Wheels.periodic();
        Pivot.periodic();
        Logger.recordOutput("3D_Feild/0_Zero", new Pose3d[] {new Pose3d(0,0,0, new Rotation3d(0, 0, -0))});
        Logger.recordOutput("3D_Feild/1_Intake_Wheels", new Pose3d[] {new Pose3d(0.04445,-0.127,0.6223, new Rotation3d(0, 0, Wheels.getPosition().in(Radians)))});
        Logger.recordOutput("3D_Feild/2_Intake_Pivot", new Pose3d[] {new Pose3d(-0.127,0,0.1143, new Rotation3d(0, Pivot.getPosition().in(Radian), 0))});
    }

    public Command setSetpoint (IntakeOptions Setpoint) {
    
        return this.run(
        ()-> {

        Wheels.runVelocity(Setpoint.getWheelsspeed().getSetpoint(),
        SingleMotorConstants.ACCELERATION, PIDSlot.SLOT_1);

        Pivot.runPosition(Setpoint.getPivotangle().getSetpoint(),
        DoubleMotorConstants.CRUISE_VELOCITY,DoubleMotorConstants.ACCELERATION,
        DoubleMotorConstants.JERK, PIDSlot.SLOT_0);
        
        }
    );
    }



}
