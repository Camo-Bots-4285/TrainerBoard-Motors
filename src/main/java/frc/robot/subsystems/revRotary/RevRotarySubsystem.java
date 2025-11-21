// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.revRotary;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.lib.W8.util.LoggedTunableNumber;
import frc.lib.W8.util.LoggerHelper;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class RevRotarySubsystem extends SubsystemBase {

    private final RotaryMechanism io;

    private static final LoggedTunableNumber STOW_SETPOINT = new LoggedTunableNumber("TEST", 0.0);
    private static final LoggedTunableNumber RAISED_SETPOINT =
        new LoggedTunableNumber("RAISED", 90);

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum Setpoint {
        STOW(Degrees.of(STOW_SETPOINT.get())),
        RAISED(Degrees.of(RAISED_SETPOINT.get()));

        private final Angle setpoint;
    }


    public RevRotarySubsystem(RotaryMechanism io)
    {
        this.io = io;

        //setSetpoint(RevRotarySubsystemConstants.DEFAULT_SETPOINT).ignoringDisable(true).schedule();
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(RevRotarySubsystemConstants.NAME, this);
        io.periodic();

    }
       

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(),
                RevRotarySubsystemConstants.CRUISE_VELOCITY,
                RevRotarySubsystemConstants.ACCELERATION, RevRotarySubsystemConstants.JERK,
                PIDSlot.SLOT_0))
            .withName("Go To "  + setpoint.name() + " Setpoint");
    };

    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, RevRotarySubsystemConstants.TOLERANCE);
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
                RevRotarySubsystemConstants.CRUISE_VELOCITY,
                RevRotarySubsystemConstants.ACCELERATION, RevRotarySubsystemConstants.JERK,
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
        //         double center = (RevRotarySubsystemConstants.MIN_ANGLE.in(Rotation) +
        //                             RevRotarySubsystemConstants.MAX_ANGLE.in(Rotation)) / 2;
        //         double amp = (-RevRotarySubsystemConstants.MIN_ANGLE.in(Rotation) +
        //                             RevRotarySubsystemConstants.MAX_ANGLE.in(Rotation)) / 2;                    
    
        //         double position = amp * -Math.cos(t%(2*Math.PI)) + center;
    
        //         io.runPosition(
        //             Rotation.of(position),
        //             RevRotarySubsystemConstants.CRUISE_VELOCITY,
        //             RevRotarySubsystemConstants.ACCELERATION,
        //             RevRotarySubsystemConstants.JERK,
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
