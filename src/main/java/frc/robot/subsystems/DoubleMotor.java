package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.lib.W8.util.LoggerHelper;

import frc.robot.Constants.DoubleMotorConstants;
import frc.robot.Constants.DoubleMotorConstants.Setpoint;


public class DoubleMotor extends SubsystemBase {

    private final RotaryMechanism io;

    public DoubleMotor(RotaryMechanism io)
    {
        this.io = io;

        setSetpoint(DoubleMotorConstants.DEFAULT_SETPOINT).ignoringDisable(true).schedule();
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(DoubleMotorConstants.NAME, this);
        io.periodic();
    }
       

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(),
                DoubleMotorConstants.CRUISE_VELOCITY,
                DoubleMotorConstants.ACCELERATION, DoubleMotorConstants.JERK,
                PIDSlot.SLOT_0))
            .withName("Go To "  + setpoint.name() + " Setpoint");
    };

    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, DoubleMotorConstants.TOLERANCE);
    }

    // public Command waitUntilGoalCommand(Angle position)
    // {
    //     return Commands.waitUntil(() -> {
    //         return nearGoal(position);
    //     });
    // }

    public Command setGoalCommandWithWait(Setpoint setpoint)
    {
        // return waitUntilGoalCommand(setpoint.getSetpoint())
        //     .deadlineFor(setSetpoint(setpoint))
        //     .withName("Go To " + setpoint.toString() + " Setpoint with wait");

        return setSetpoint(setpoint)
        .until( ()-> nearGoal(setpoint.getSetpoint()))
        .withName("Go To " + setpoint.toString() + " Setpoint with wait");
    }

    private double startTime;
    public Command setWiggle() {
        return this.runOnce(
            () -> io.runPosition(Rotation.of(0.35),
                DoubleMotorConstants.CRUISE_VELOCITY,
                DoubleMotorConstants.ACCELERATION, DoubleMotorConstants.JERK,
                PIDSlot.SLOT_1))
            .withName("Go To "  + " Setpoint");

        // return new FunctionalCommand(
        //     // init: runs ONCE at command start
        //     () -> {
        //         startTime = Timer.getFPGATimestamp();    // store starting offset
        //     },
    
        //     // execute: runs repeatedly
        //     () -> {
        //         double t = Timer.getFPGATimestamp() - startTime; // <-- zeroed time
        //         double center = (DoubleMotorConstants.MIN_ANGLE.in(Rotation) +
        //                             DoubleMotorConstants.MAX_ANGLE.in(Rotation)) / 2;
        //         double amp = (-DoubleMotorConstants.MIN_ANGLE.in(Rotation) +
        //                             DoubleMotorConstants.MAX_ANGLE.in(Rotation)) / 2;                    
    
        //         double position = amp * -Math.cos(t%(2*Math.PI)) + center;
    
        //         io.runPosition(
        //             Rotation.of(position),
        //             DoubleMotorConstants.CRUISE_VELOCITY,
        //             DoubleMotorConstants.ACCELERATION,
        //             DoubleMotorConstants.JERK,
        //             PIDSlot.SLOT_0
        //         );
        //     },
    
        //     // end: runs ONCE when the command is interrupted or finishes
        //     (interrupted) -> {
        //         // Optional: stop servo or go to neutral
        //         // io.runPosition(Rotation.of(variation), ...);
        //     },
    
        //     // isFinished: normally false → runs until interrupted
        //     () -> false,
    
        //     this // subsystem requirement
        // ).withName("Servo Wiggle");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }

}
