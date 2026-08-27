package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;



import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.Shooter_Flywheel_Constants;



public class Shooter extends SubsystemBase {
    private final FlywheelMechanism <?> FlyWheel;

public Shooter(FlywheelMechanism<?>  FlyWheel)
{

this.FlyWheel = FlyWheel;
}


@Override
    public void periodic()
    {

       
        FlyWheel.periodic();

    }


public Command SetIdle()
{
    return this.run(
        ()-> {
           // targetFlywheelSpeed = Shooter_Flywheel_Constants.IDLE_VELOCITY;

            FlyWheel.runCoast();



        }





    ).withName("Set Idle");      


    }

public Command setFirebob()
{

return
    this.run(
        () -> {

            FlyWheel.runVelocity(RotationsPerSecond.of(75),PIDSlot.SLOT_1);
        }

    ).withName("setFire");

}

}