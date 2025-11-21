package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motors extends SubsystemBase{
    public SparkFlex Leader;
    public SparkClosedLoopController controller;
    private RelativeEncoder encoder;

    public Motors(){
        Leader = new SparkFlex(1, MotorType.kBrushless);
         controller = Leader.getClosedLoopController();
         encoder = Leader.getEncoder();

        // Leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // @Override
    // public void periodic(){
    //     //Leader.set(0.05);
    //     //System.out.println("Positon"+encoder.getPosition());
    // }


    public Command MotorSpin(double speed){
        return this.run(()->Leader.set(speed)).withName("Turbo like the snail");
    }
}
