package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.Drive;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.Shooter_Flywheel_Constants;



public class Shooter extends SubsystemBase {
    private final FlywheelMechanism <?> FlyWheel;

private AngularVelocity targetFlywheelSpeed = RotationsPerSecond.of(0);
public Shooter(FlywheelMechanism Flywheel)
{

this.FlyWheel = FlyWheel






}

public Command SetIdle()
{
    return this.run(
        ()-> {
            targetFlywheelSpeed = Shooter_Flywheel_Constants.IDLE_VELOCITY;

            FlyWheel.runCoast();



        }





    ).withName("Set Idle");      


    }

public Command setAim(Supplier<Angle> rawPitch,Supplier<Angle> rawYaw, Supplier<AngularVelocity> speed)
{
    return
    this.run(
        () -> {

        Angle angle = rawPitch.get();




        }




    )







}












}
