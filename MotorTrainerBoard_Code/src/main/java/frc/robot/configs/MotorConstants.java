package frc.robot.configs;

/**
 * Contains motor configuration constants and utility methods for generating
 * motor parameter arrays specific to different motor types used in the robot.
 * 
 * This class provides:
 * - Current limits, RPM limits, emergency limits for various motor models.
 * - Gear ratio, wheel radius, and temperature warning thresholds.
 * - Methods to create standardized configuration arrays for motion profiles.
 * 
 * All motor configuration methods return arrays of integers or doubles
 * representing key parameters to be used in motor controllers.
 * 
 * Supported motors include:
 * - Neo
 * - NeoVortex
 * - Neo550
 * - Spark Minion
 * - Kraken60 (Talon CTRE)
 * - Kraken44 (Talon CTRE)
 * - Falcon (Talon CTRE)
 * 
 * Motion profile arrays are also generated for REV and CTRE motor controllers.
 */
public class MotorConstants {

    /**
     * Returns integer parameters for a NEO motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return An int array with:
     *         [0] Motor ID,
     *         [1] Stall Current Limit (Amps at 0 RPM),
     *         [2] Free Spin Current Limit (Amps at free speed),
     *         [3] Limit RPM threshold for scaling current limit,
     *         [4] Emergency Current Limit.
     */
    public static int[] Neo_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            40,
            50,
            100,
            60
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a NEO motor.
     * 
     * @param Gear_Ratio Gear ratio of the drivetrain subsystem.
     * @param Wheel_Radius Radius of the wheel in units consistent with robot control.
     * @return A double array with:
     *         [0] Gear ratio,
     *         [1] Wheel radius,
     *         [2] Temperature overheat warning threshold in Celsius.
     */
    public static double[] Neo_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Returns integer parameters for a NeoVortex motor (similar to NEO).
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array structured like Neo_int.
     */
    public static int[] NeoVortex_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            40,
            50,
            100,
            60
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a NeoVortex motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array structured like Neo_double.
     */
    public static double[] NeoVortex_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Returns integer parameters for a Neo550 motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array with stall current limit and other parameters specific to Neo550.
     */
    public static int[] Neo550_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            35,
            45,
            100,
            50
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for Neo500 motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array structured like Neo_double.
     */
    public static double[] Neo500_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Returns integer parameters for a Spark Minion motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array with current limits and RPM thresholds for Spark Minion.
     */
    public static int[] Spark_Minion_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            35,
            45,
            100,
            50
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a Spark Minion motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array structured like Neo_double.
     */
    public static double[] Spark_Minion_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Creates a MotionProfile configuration array with PID and feedforward parameters for position,
     * velocity, and voltage control loops, along with motion constraints.
     * 
     * The array contains 27 elements organized as follows:
     * 
     * Indices 0–7:   Position Control PID + Feedforward
     * Indices 8–15:  Velocity Control PID + Feedforward
     * Indices 16–23: Voltage Control PID + Feedforward
     * Indices 24–26: Motion Constraints
     * 
     * @param pos_kP        Position PID proportional gain
     * @param pos_kI        Position PID integral gain
     * @param pos_kD        Position PID derivative gain
     * @param pos_kV        Position feedforward velocity gain (kV)
     * @param pos_iZone     Position integral zone (iZone)
     * @param pos_iAccum    Position maximum integral accumulation
     * @param pos_minOut    Position controller minimum output
     * @param pos_maxOut    Position controller maximum output
     * @param vel1_kP       Velocity PID proportional gain
     * @param vel1_kI       Velocity PID integral gain
     * @param vel1_kD       Velocity PID derivative gain
     * @param vel1_kV       Velocity feedforward velocity gain (kV)
     * @param vel1_iZone    Velocity integral zone (iZone)
     * @param vel1_iAccum   Velocity maximum integral accumulation
     * @param vel1_minOut   Velocity controller minimum output
     * @param vel1_maxOut   Velocity controller maximum output
     * @param volt_kP       Voltage PID proportional gain
     * @param volt_kI       Voltage PID integral gain
     * @param volt_kD       Voltage PID derivative gain
     * @param volt_kV       Voltage feedforward velocity gain (kV)
     * @param volt_iZone    Voltage integral zone (iZone)
     * @param volt_iAccum   Voltage maximum integral accumulation
     * @param volt_minOut   Voltage controller minimum output
     * @param volt_maxOut   Voltage controller maximum output
     * @param maxVelocity   Maximum motion profile velocity (units per second)
     * @param maxAcceleration Maximum motion profile acceleration (units per second squared)
     * @param allowedError  Allowed closed-loop error (units)
     * @return A 27-element double array representing the motion profile configuration.
     */
    public static double[] REV_createMotionProfile(
        double pos_kP, double pos_kI, double pos_kD, double pos_kV, double pos_iZone, double pos_iAccum, double pos_minOut, double pos_maxOut,
        double vel1_kP, double vel1_i, double vel1_kD, double vel1_kV, double vel1_iZone, double vel1_iAccum, double vel1_minOut, double vel1_maxOut,
        double volt_kP, double volt_kI, double volt_kD, double volt_kV, double volt_iZone, double volt_iAccum, double volt_minOut, double volt_maxOut,
        double maxVelocity, double maxAcceleration, double allowedError
    ) {
        double[] profile = new double[27];

        // Position PID + Feedforward
        profile[0] = pos_kP;
        profile[1] = pos_kI;
        profile[2] = pos_kD;
        profile[3] = pos_kV;
        profile[4] = pos_iZone;
        profile[5] = pos_iAccum;
        profile[6] = pos_minOut;
        profile[7] = pos_maxOut;

        // Velocity PID + Feedforward
        profile[8]  = vel1_kP;
        profile[9]  = vel1_i;
        profile[10] = vel1_kD;
        profile[11] = vel1_kV;
        profile[12] = vel1_iZone;
        profile[13] = vel1_iAccum;
        profile[14] = vel1_minOut;
        profile[15] = vel1_maxOut;

        // Voltage PID + Feedforward (third group)
        profile[16] = volt_kP;
        profile[17] = volt_kI;
        profile[18] = volt_kD;
        profile[19] = volt_kV;
        profile[20] = volt_iZone;
        profile[21] = volt_iAccum;
        profile[22] = volt_minOut;
        profile[23] = volt_maxOut;

        // Motion constraints
        profile[24] = maxVelocity;
        profile[25] = maxAcceleration;
        profile[26] = allowedError;

        return profile;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////                    CTRE Rection Starts Now                         ////
    ////////////////////////////////////////////////////////////////////////////
   
    /**
     * Returns integer parameters for a Kraken60 Talon motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array with stall current limit and other parameters.
     */
    public static int[] Kraken60_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            55,
            65,
            500,
            80
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a Kraken60 Talon motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array with gear ratio, wheel radius, temperature threshold.
     */
    public static double[] Kraken60_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Returns integer parameters for a Kraken44 Talon motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array with stall current limit and other parameters.
     */
    public static int[] Kraken44_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            50,
            60,
            350,
            70
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a Kraken44 Talon motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array with gear ratio, wheel radius, temperature threshold.
     */
    public static double[] Kraken44_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

    /**
     * Returns integer parameters for a Falcon Talon motor.
     * 
     * @param Motor_ID The CAN ID of the motor.
     * @return Int array with stall current limit and other parameters.
     */
    public static int[] Falcon_int(int Motor_ID) {
        int[] Motor_int = {
            Motor_ID,
            60,
            70,
            580,
            90
        };
        return Motor_int;
    }

    /**
     * Returns double parameters for a Falcon Talon motor.
     * 
     * @param Gear_Ratio Gear ratio.
     * @param Wheel_Radius Wheel radius.
     * @return Double array with gear ratio, wheel radius, temperature threshold.
     */
    public static double[] Falcon_double(double Gear_Ratio, double Wheel_Radius) {
        double[] Motor_double = {
            Gear_Ratio,
            Wheel_Radius,
            70
        };
        return Motor_double;
    }

/**
 * Creates a motion profile configuration array for CTRE Talon motor controllers.
 * 
 * The profile contains PID gains, feedforward parameters, motion constraints, and feedforward options.
 * 
 * @param pos_kP        Position PID proportional gain
 * @param pos_kI        Position PID integral gain
 * @param pos_kD        Position PID derivative gain
 * @param pos_kS        Position feedforward static gain (kS)
 * @param pos_kV        Position feedforward velocity gain (kV)
 * @param pos_kA        Position feedforward acceleration gain (kA)
 * @param reverseFF     Reverse feedforward flag (0 = false, 1 = true)
 * @param ffType        Feedforward type:
 *                      0 = Normal (force opposes motion),
 *                      1 = Constant (e.g., elevator),
 *                      2 = Varies with position (e.g., pivot/arm)
 * @param vel_kP        Velocity PID proportional gain
 * @param vel_kI        Velocity PID integral gain
 * @param vel_kD        Velocity PID derivative gain
 * @param vel_kS        Velocity feedforward static gain (kS)
 * @param vel_kV        Velocity feedforward velocity gain (kV)
 * @param vel_kA        Velocity feedforward acceleration gain (kA)
 * @param cruiseVelocity Maximum cruise velocity (units/sec)
 * @param acceleration  Maximum acceleration (units/sec^2)
 * @param jerk          Maximum jerk (units/sec^3)
 * @param motionKV      Motion profile velocity gain
 * @param motionKA      Motion profile acceleration gain
 * @return A 19-element double array representing the CTRE motion profile configuration.
 */
public static double[] CTRE_createMotionProfile(
    double pos_kP, double pos_kI, double pos_kD, double pos_kS, double pos_kV, double pos_kA, double reverseFF, double ffType,
    double vel_kP, double vel_kI, double vel_kD, double vel_kS, double vel_kV, double vel_kA,
    double cruiseVelocity, double acceleration, double jerk, double motionKV, double motionKA
) {
    double[] profile = new double[19];

    // Position PID + Feedforward + FF options
    profile[0] = pos_kP;
    profile[1] = pos_kI;
    profile[2] = pos_kD;
    profile[3] = pos_kS;
    profile[4] = pos_kV;
    profile[5] = pos_kA;
    profile[6] = reverseFF;
    profile[7] = ffType;

    // Velocity PID + Feedforward
    profile[8] = vel_kP;
    profile[9] = vel_kI;
    profile[10] = vel_kD;
    profile[11] = vel_kS;
    profile[12] = vel_kV;
    profile[13] = vel_kA;

    // Motion Constraints + profile feedforward gains
    profile[14] = cruiseVelocity;
    profile[15] = acceleration;
    profile[16] = jerk;
    profile[17] = motionKV;
    profile[18] = motionKA;

    return profile;
}

}
