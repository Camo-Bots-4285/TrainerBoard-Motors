package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeOptions;

public class Intake extends SubsystemBase{

    private final FlywheelMechanism<?> Wheels;
    private final FlywheelMechanism<?> Pivot;

    public Intake(FlywheelMechanism<?> Wheels,FlywheelMechanism<?> Pivot)
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

        Logger.recordOutput("3D_Feild/0_2022_RobotPose", new Pose3d[] {new Pose3d(3.75,3.25,0.075, new Rotation3d(0, 0, 0))});
        Logger.recordOutput("3D_Feild/1_2022_Turret_Yaw", new Pose3d[] {new Pose3d(-0.0762,0,0, new Rotation3d(0, 0, Wheels.getPosition().in(Radians)))});
        Logger.recordOutput("3D_Feild/2_2022_Turret_Pitch", new Pose3d[] {new Pose3d(-0.0775-0.0975*Math.cos(Wheels.getPosition().in(Radians)),-0.0975*Math.sin(Wheels.getPosition().in(Radians)),0.48, new Rotation3d(0, Pivot.getPosition().in(Radian), Wheels.getPosition().in(Radians)))});
        Logger.recordOutput("3D_Feild/3_2022_Intake", new Pose3d[] {new Pose3d(0.19,0,0.37, new Rotation3d(0, Pivot.getPosition().in(Radian), 0))});
    }

    public Command setSetpoint (IntakeOptions Setpoint) {
    
        return this.run(
        ()-> {

        Wheels.runVelocity(Setpoint.getWheelsspeed().getSetpoint(), PIDSlot.SLOT_3);

        Pivot.runUnprofiledPosition(Setpoint.getPivotangle().getSetpoint(), PIDSlot.SLOT_0);
        
        }
    ).withName("Go to " + Setpoint);
    }



}
