package frc.robot.lib.Motor;

import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * MotionProfileHelpers
 * 
 * <p> Helper class which has multiple classes and methods to help the readablity and function of 
 * {@link frc.robot.lib.Motor.MotorBase MotorBase}
 * 
 * <p>Features:
 * <p>- Sort motion profile values that can be acress as varbales and not array numbers
 * <p>- Methods that return a MotionProfile of all zero effectivly disabling app motion profiled control if null value is pass
 * 
 * <p>Hardware:
 * <p>- Can be used with both CTRE and Rev
 * 
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class MotionProfileHelpers {
  // === REV ===
  // <editor-fold>

    /**
     * Hold all values used in REV motion profiling as varibale to increase readability
     */
    public static class REV_MotionProfile {
        public double kP_Pos, kI_Pos, kD_Pos, kV_Pos;
        public double iZone_Pos, iMaxAccum_Pos, outMin_Pos, outMax_Pos;

        public double kP_Vel, kI_Vel, kD_Vel, kV_Vel;
        public double iZone_Vel, iMaxAccum_Vel, outMin_Vel, outMax_Vel;

        public double kP_Alt, kI_Alt, kD_Alt, kV_Alt;
        public double iZone_Alt, iMaxAccum_Alt, outMin_Alt, outMax_Alt;

        public double cruiseRPM, accelRPMPerSec, allowedErrorRotations;

        public REV_MotionProfile(
            double kP_Pos, double kI_Pos, double kD_Pos, double kV_Pos,
            double iZone_Pos, double iMaxAccum_Pos, double outMin_Pos, double outMax_Pos,
            double kP_Vel, double kI_Vel, double kD_Vel, double kV_Vel,
            double iZone_Vel, double iMaxAccum_Vel, double outMin_Vel, double outMax_Vel,
            double kP_Alt, double kI_Alt, double kD_Alt, double kV_Alt,
            double iZone_Alt, double iMaxAccum_Alt, double outMin_Alt, double outMax_Alt,
            double cruiseRPM, double accelRPMPerSec, double allowedErrorRotations
        ) {
            this.kP_Pos = kP_Pos; this.kI_Pos = kI_Pos; this.kD_Pos = kD_Pos; this.kV_Pos = kV_Pos;
            this.iZone_Pos = iZone_Pos; this.iMaxAccum_Pos = iMaxAccum_Pos; this.outMin_Pos = outMin_Pos; this.outMax_Pos = outMax_Pos;
            this.kP_Vel = kP_Vel; this.kI_Vel = kI_Vel; this.kD_Vel = kD_Vel; this.kV_Vel = kV_Vel;
            this.iZone_Vel = iZone_Vel; this.iMaxAccum_Vel = iMaxAccum_Vel; this.outMin_Vel = outMin_Vel; this.outMax_Vel = outMax_Vel;
            this.kP_Alt = kP_Alt; this.kI_Alt = kI_Alt; this.kD_Alt = kD_Alt; this.kV_Alt = kV_Alt;
            this.iZone_Alt = iZone_Alt; this.iMaxAccum_Alt = iMaxAccum_Alt; this.outMin_Alt = outMin_Alt; this.outMax_Alt = outMax_Alt;
            this.cruiseRPM = cruiseRPM; this.accelRPMPerSec = accelRPMPerSec; this.allowedErrorRotations = allowedErrorRotations;
        }
    }

    /**
     * @return a all zero instance of REV_MotionProfile so if null value is pass motion profile based method will be disabled
     */
    public static REV_MotionProfile getDisabledREVProfile() {
        return new REV_MotionProfile(
            // Slot 0: Position PID + FF
            0, 0, 0, 0,
            0, 0, 0, 0,
    
            // Slot 1: Velocity PID + FF
            0, 0, 0, 0,
            0, 0, 0, 0,
    
            // Slot 2: Alternate PID + FF
            0, 0, 0, 0,
            0, 0, 0, 0,
    
            // Motion constraints
            0, 0, 0
        );
    }

  // </editor-fold>

  // === CTRE ===
  // <editor-fold>

    /**
     * Hold all values used in CTRE motion profiling as varibale to increase readability
     */
    public static class CTRE_MotionProfile {
        public double kP_Pos, kI_Pos, kD_Pos, ff_kS, ff_kV, ff_kA;
        public GravityTypeValue gravityType; // replaces int ffType
        public double kP_Vel, kI_Vel, kD_Vel, velFF_kS, velFF_kV, velFF_kA;
        public double cruiseVelocity, acceleration, jerk, motionKV, motionKA;

        public CTRE_MotionProfile(
            double kP_Pos, double kI_Pos, double kD_Pos,
            double ff_kS, double ff_kV, double ff_kA,
            double gravityTypeCode,
            double kP_Vel, double kI_Vel, double kD_Vel,
            double velFF_kS, double velFF_kV, double velFF_kA,
            double cruiseVelocity, double acceleration, double jerk,
            double motionKV, double motionKA
        ) {
            this.kP_Pos = kP_Pos; this.kI_Pos = kI_Pos; this.kD_Pos = kD_Pos;
            this.ff_kS = ff_kS; this.ff_kV = ff_kV; this.ff_kA = ff_kA;

            if (gravityTypeCode == 1) {
                this.gravityType = GravityTypeValue.Elevator_Static;
            } else if (gravityTypeCode == 2) {
                this.gravityType = GravityTypeValue.Arm_Cosine;
            } else {
                this.gravityType = null;
            }

            this.kP_Vel = kP_Vel; this.kI_Vel = kI_Vel; this.kD_Vel = kD_Vel;
            this.velFF_kS = velFF_kS; this.velFF_kV = velFF_kV; this.velFF_kA = velFF_kA;
            this.cruiseVelocity = cruiseVelocity; this.acceleration = acceleration; this.jerk = jerk;
            this.motionKV = motionKV; this.motionKA = motionKA;
        }
    }

    /**
     * @return a all zero instance of CTRE_MotionProfile so if null value is pass motion profile based method will be disabled
     */
    public static CTRE_MotionProfile getDisabledCTREProfile() {
        return new CTRE_MotionProfile(
            // Position PID gains
            0, 0, 0,
            // Feedforward kS, kV, kA
            0, 0, 0,
            // Gravity type code (0 = none)
            0,
            // Velocity PID gains
            0, 0, 0,
            // Velocity feedforward kS, kV, kA
            0, 0, 0,
            // Motion constraints: cruiseVelocity, acceleration, jerk
            0, 0, 0,
            // Motion feedforward constants
            0, 0
        );
    }
    // </editor-fold>

    // === SIM ===
    // <editor-fold>
    public static class SIM_MotionProfile {
        // PID + Feedforward for Position Control
        public double kP_Pos, kI_Pos, kD_Pos, kS_Pos, kV_Pos, kA_Pos;
    
        // PID + Feedforward for Velocity Control
        public double kP_Vel, kI_Vel, kD_Vel, kS_Vel, kV_Vel, kA_Vel;
    
        public double kG;
        // Motion Constraints
        public double cruiseRPM, accelRPMPerSec;
    
        public SIM_MotionProfile(
            // Position Control PID + FF
            double kP_Pos, double kI_Pos, double kD_Pos, double kS_Pos, double kV_Pos, double kA_Pos, 
    
            // Velocity Control PID + FF
            double kP_Vel, double kI_Vel, double kD_Vel, double kS_Vel, double kV_Vel, double kA_Vel,
    
            double kG,
            // Motion Constraints
            double cruiseRPM, double accelRPMPerSec
        ) {
            // Position Control Values
            this.kP_Pos = kP_Pos;
            this.kI_Pos = kI_Pos;
            this.kD_Pos = kD_Pos;
            this.kS_Pos = kS_Pos;
            this.kV_Pos = kV_Pos;
            this.kA_Pos = kA_Pos;

            // Velocity Control Values
            this.kP_Vel = kP_Vel;
            this.kI_Vel = kI_Vel;
            this.kD_Vel = kD_Vel;
            this.kS_Vel = kS_Vel;
            this.kV_Vel = kV_Vel;
            this.kA_Vel = kA_Vel;

            // Motion Constraints
            this.cruiseRPM = cruiseRPM;
            this.accelRPMPerSec = accelRPMPerSec;
        }
    }

    // Default motion profile with all zeros
    public static SIM_MotionProfile getSIMDefaultProfile() {
        return new SIM_MotionProfile(
            // Position Control PID + FF
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 

            // Velocity Control PID + FF
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

            0.0,

            // Motion Constraints
            0.0, 0.0
        );
    }

  // </editor-fold>

}
