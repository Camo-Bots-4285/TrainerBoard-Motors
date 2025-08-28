/**
 * Copyright (c) 2021-2025 Littleton Robotics. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this list of conditions, and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of Littleton Robotics, FRC 6328 ("Mechanical Advantage"), AdvantageKit, nor the names of other AdvantageKit contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY LITTLETON ROBOTICS AND OTHER ADVANTAGEKIT CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LITTLETON ROBOTICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The use of this source code is governed by a BSD license that can be found in the LICENSE file at the root directory of this project.
 *
 * For more information, visit: 
 * http://github.com/Mechanical-Advantage
 */

package frc.robot.lib.MotorV3;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface representing motor control and sensor updates for various motor types. 
 * This interface provides a structure for motor input/output operations such as 
 * updating sensor data, setting voltage or velocity, and configuring advanced motor settings.
 * 
 * <p>The {@link MotorIO} interface is designed to provide standard methods for controlling 
 * and monitoring the status of motors, whether they are configured for position control, 
 * velocity control, or other motor-specific tasks.</p>
 * 
 * @author CD
 * @since Summer of 2025
 */
public interface MotorIO {
    
    // Input structure to update sensor readings
    @AutoLog
    public class MotorIOInputs {
        /** The current motor position in rotations. */
        public double motorPositionRot = 0.0;
        
        /** The current motor velocity in RPM (revolutions per minute). */
        public double motorVelocityRPM = 0.0;
        
        /** The current mechanism position in rotations (e.g., for an arm or wheel). */
        public double mechanismPositionRot = 0.0;
        
        /** The current mechanism velocity in RPM. */
        public double mechanismVelocityRPM = 0.0;
        
        /** The tip speed in meters per second (used for calculating velocity at the edge of a wheel). */
        public double tipSpeedMps = 0.0;
        
        /** The applied voltage to the motor in volts. */
        public double appliedVolts = 0.0;
        
        /** The current in amps being drawn by the motor. */
        public double amps = 0.0;
        
        /** The motor's temperature in Celsius. */
        public double temperatureCelsius = 0.0;
    }

    /**
     * Updates the input readings for the motor and mechanism. This method will be called 
     * to refresh the motor's sensor data, such as position, velocity, and applied voltage.
     *
     * @param inputs The {@link MotorIOInputs} object containing updated sensor values.
     */
    void updateInputs(MotorIOInputs inputs);

    /**
     * Sets the motor voltage directly. This method will set the voltage applied to the motor.
     * 
     * @param volts The voltage to apply to the motor in volts.
     */
    void setVoltage(double volts);

    /**
     * Sets the motor's output as a percentage of the maximum voltage.
     * 
     * @param percent The percentage of the maximum voltage to apply (from -100% to 100%).
     */
    void setPercent(double percent);

    /**
     * Sets the target motor position to the specified number of motor rotations.
     * 
     * @param motorRotations The desired motor position in rotations.
     */
    void setTargetMotorPosition(double motorRotations);

    /**
     * Sets the motor's current position. This method is typically used to reset the motor's position.
     * 
     * @param motorRotations The current motor position in rotations.
     */
    void setCurrentMotorPosition(double motorRotations);

    /**
     * Sets the target position of the mechanism (e.g., arm or lift system) in rotations.
     * 
     * @param mechanismRotations The desired mechanism position in rotations.
     */
    void setTargetMechanismPosition(double mechanismRotations);

    /**
     * Sets the mechanism's current position. Similar to setting the motor's current position.
     * 
     * @param mechanismRotations The current mechanism position in rotations.
     */
    void setCurrentMechanismPosition(double mechanismRotations);

    /**
     * Sets the target motor velocity (RPM).
     * 
     * @param motorRPM The desired motor velocity in revolutions per minute.
     */
    void setTargetMotorVelocity(double motorRPM);

    /**
     * Sets the target mechanism velocity (RPM).
     * 
     * @param mechanismRPM The desired mechanism velocity in revolutions per minute.
     */
    void setTargetMechanismVelocity(double mechanismRPM);

    /**
     * Sets the target tip speed in meters per second (useful for calculating wheel or arm speed at the edge).
     * 
     * @param mps The desired tip speed in meters per second.
     */
    void setTargetTipSpeed(double mps);

    /**
     * Stops the motor and any controlled mechanism immediately.
     */
    void stop();

    /**
     * Sets the motor to brake or coast mode.
     * 
     * @param isBrake If true, the motor will engage brake mode; if false, it will engage coast mode.
     */
    default void setBrakeMode(boolean isBrake){
                // Default no-op: Implement this in specific motor classes if needed
    }

    /**
     * Sets advanced tuning constants for the motor, such as PID values and feedforward constants.
     * This method allows the user to tune the motor control system for different behaviors.
     * 
     * @param p Proportional constant (P) for PID control.
     * @param i Integral constant (I) for PID control.
     * @param d Derivative constant (D) for PID control.
     * @param ff Feedforward constant for control.
     * @param iZone The integral zone for the PID controller.
     * @param iMaxAccum Maximum integral accumulation for PID.
     * @param minOutput Minimum output allowed for PID.
     * @param maxOutput Maximum output allowed for PID.
     * @param slot The slot for PID settings (use different slots for different modes).
     */
    default void setTunerConstants(double p, double i, double d, double ff,
                                   double iZone, double iMaxAccum,
                                   double minOutput, double maxOutput, int slot) {
        // Default no-op: Implement this in specific motor classes if needed
    }

    /**
     * Sets motion constraints for the motor, including maximum velocity and acceleration, 
     * as well as allowed error thresholds for closed-loop control.
     * 
     * @param maxVel The maximum velocity for the motor in RPM.
     * @param maxAccel The maximum acceleration for the motor in RPM per second.
     * @param allowedError The allowed error in position for closed-loop control.
     * @param slot The slot for motion settings (use different slots for different modes).
     */
    default void setMotionConstraints(double maxVel, double maxAccel,
                                      double allowedError, int slot) {
        // Default no-op: Implement this in specific motor classes if needed
    }

    /**
     * Sets the current limits for the motor, including stall current, free current, and RPM limit.
     * 
     * @param stallLimit The maximum stall current in amps.
     * @param freeLimit The maximum free current in amps.
     * @param limitRPM The maximum RPM the motor can reach.
     */
    default void setAmpLimits(int stallLimit, int freeLimit, int limitRPM) {
        // Default no-op: Implement this in specific motor classes if needed
    }

    /**
     * Sets the software to not be able to go past a max and min value
     * @param max The maximum value the mechanism should move
     * @param min The minimum value the mechanism should move
     */
    default void setMotionLimit (Double max, Double min){
        // Default no-op: Implement this in specific motor classes if needed
    }
}
