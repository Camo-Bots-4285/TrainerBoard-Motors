package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DoubleMotorConstants.Setpoint;
import frc.robot.Constants.IntakeConstants.IntakeOptions;

public class Turret extends SubsystemBase{

    private final FlywheelMechanism<?> Yaw;
    private final FlywheelMechanism<?> Pitch;

    public Turret(FlywheelMechanism<?> Yaw,FlywheelMechanism<?> Pitch)
    {
        this.Yaw = Yaw;
        this.Pitch = Pitch;
    }

@Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(IntakeConstants.NAME, this);
        Yaw.periodic();
        Pitch.periodic();

            }

   
public Command setPoint (Angle target){
 return this.run(
    ()-> {

        Yaw.runUnprofiledPosition(target,PIDSlot.SLOT_0);

    }
 ).withName("Go to" + target.in(Rotations));


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



public Command setAim(Supplier<Angle> rawPitch, Supplier<Angle> rawYaw,Supplier<Angle> pose2d){


    return this.run(
        ()-> {

            Angle Yaw_ = turretFilter(rawYaw.get());
            Angle Pitch_ = rawPitch.get();

           Yaw.runUnprofiledPosition(Yaw_, PIDSlot.SLOT_0);
            Pitch.runUnprofiledPosition(Pitch_, PIDSlot.SLOT_0);

            AngleUnit degrees;
            
                    
            
                    
                    
                    }
            
            
            
                );
            
            }
            
            
            public Command lookFowards(Supplier<Angle> rawPitch, Supplier<Angle> rawYaw, Supplier<Angle> pose2d){


 return this.run(
    ()-> {
             Angle Yaw_ = turretFilter(rawYaw.get());
            Angle Pitch_ = rawPitch.get();


        








    }

 







 );




 }







}
