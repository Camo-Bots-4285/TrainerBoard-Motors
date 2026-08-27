package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants.IntakeConstants;

public class Turret extends SubsystemBase{

    private final FlywheelMechanism<?> Yaw;
    private final FlywheelMechanism<?> Pitch;
    private final FlywheelMechanism<?> FlyWheel;
   
    public Turret(FlywheelMechanism<?> Yaw,FlywheelMechanism<?> Pitch, FlywheelMechanism<?> flyWheel)
    {
        this.Yaw = Yaw;
        this.Pitch = Pitch;
        this.FlyWheel = flyWheel;
    }

@Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(IntakeConstants.NAME, this);
        Yaw.periodic();
        Pitch.periodic();
        FlyWheel.periodic();

    }

    public Command setAim(Supplier<Angle> rawPitch, Supplier<Angle> rawYaw, Supplier<AngularVelocity> flyWheelVelcoity){

        return this.run(
            ()-> {

                Angle Yaw_ = turretFilter(rawYaw.get());
                Angle Pitch_ = rawPitch.get();

                Yaw.runUnprofiledPosition(Yaw_, PIDSlot.SLOT_0);
                Pitch.runUnprofiledPosition(Pitch_, PIDSlot.SLOT_0);
                FlyWheel.runVelocity(flyWheelVelcoity.get(), PIDSlot.SLOT_1);

            }

        );

    }


public Angle turretFilter (Angle rawAngle){

Angle Wrap = Degrees.of(rawAngle.in(Degrees) % 360) ;

if(Wrap.in(Degrees) < 0 ) {
//Wrap.plus(Degrees.of(360));
Wrap = Degrees.of(Wrap.in(Degrees) + 360) ;
}
System.out.println(Wrap.toString());

return Wrap;



}

            
}
