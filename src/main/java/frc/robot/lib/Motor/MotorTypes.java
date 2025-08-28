package frc.robot.lib.Motor;

public class MotorTypes {

// Nested abstract class REVMotor inside Neo class
public static abstract class Motor_Type {
        
    // Abstract methods that must be implemented by concrete motor classes
/**
 * Abstract method to get the emergency stop current.
 * This defines the current level at which the motor should immediately stop to avoid damage.
 * 
 * @return The emergency stop current in amperes.
 */
public abstract double getEmergencyStopCurrent();

/**
 * Abstract method to get the motor's overheat temperature.
 * This defines the maximum safe operating temperature for the motor.
 * 
 * @return The overheat temperature in Celsius.
 */
public abstract double getOverHeatTemp();

/**
 * Abstract method to define the number of loops required to monitor the motor before a safety shutdown.
 * This will be used to check if the motor is operating within the safe limits.
 * 
 * @return The number of loops for monitoring the motor.
 */
public abstract int getNumLoops();

/**
 * Abstract method to get the required number of loops to shut off the motor after it exceeds safe parameters.
 * This will be used to determine how many consecutive readings are needed to trigger a shutdown.
 * 
 * @return The number of loops required to shut off the motor.
 */
public abstract int getRequiredLoopsToShutOff();

/**
 * Abstract method to get the cooldown duration after an emergency stop.
 * This is the amount of time the motor should remain off before attempting to restart.
 * 
 * @return The emergency stop cooldown duration in seconds.
 */
public abstract double getEmergencyStopCooldownDuration();


/**
 * Abstract method to retrieve the default amp limits for the motor.
 * The returned array depends on the motor type, either for **REV motors** or **CTRE motors**:
 * 
 * <p><b>For REV Motors</b> (e.g., Neo, Minion, SparkMax):</p>
 * <ul>
 *     <li>Stall current limit: The maximum current drawn when the motor is at 0 RPM.</li>
 *     <li>Free current limit: The current when the motor runs at free speed (~4500 RPM).</li>
 *     <li>RPM threshold: Below this RPM, the motor will use the stall current limit. Above this RPM, it will scale to the free current limit.</li>
 * </ul>
 *
 * <p><b>For CTRE Motors</b> (e.g., Falcon, Talon):</p>
 * <ul>
 *     <li>SupplyCurrentLimit: The maximum current delivered to the motor before limiting it.</li>
 *     <li>SupplyCurrentLowerTime: The time duration before the motor reduces its supply current.</li>
 *     <li>SupplyCurrentLowerLimit: The reduced supply current after the relaxation time.</li>
 *     <li>StatorCurrentLimit: The maximum current allowed in the stator windings of the motor (protects from overheating).</li>
 * </ul>
 * 
 * <p>The values returned by this method are motor-specific, designed to protect against overcurrent and overheating.</p>
 *
 * <p><b>Return:</b></p>
 * <ul>
 *     <li>For REV Motors:</li>
 *     <ul>
 *         <li>0 - Stall current limit (amps)</li>
 *         <li>1 - Free current limit (amps)</li>
 *         <li>2 - RPM threshold (below which the stall limit applies)</li>
 *     </ul>
 *     <li>For CTRE Motors:</li>
 *     <ul>
 *         <li>0 - Supply current limit (amps)</li>
 *         <li>1 - Time before relaxing the supply current (seconds)</li>
 *         <li>2 - Supply current after relaxation (amps)</li>
 *         <li>3 - Stator current limit (amps)</li>
 *     </ul>
 * </ul>
 */
public abstract int[] getDefaultAmpLimits();

    
}

  // === REV ===
  // <editor-fold>
    public static class Neo extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 55;
    }

    @Override
    public double getOverHeatTemp() {
        return 80;
    }

    @Override
    public int getNumLoops() {
        return 20;
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15;
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5;
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[]{
            35, // stallLimit
            20, // freeLimit
            300 // limitRpm
        };
    }   
}

    public static class Neo550 extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 40;
    }

    @Override
    public double getOverHeatTemp() {
        return 80;
    }

    @Override
    public int getNumLoops() {
        return 20;
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15;
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5;
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[]{
            25, // stallLimit
            15, // freeLimit
            400 // limitRpm
        };
    }
}

    public static class Minion extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 40;
    }

    @Override
    public double getOverHeatTemp() {
        return 85;
    }

    @Override
    public int getNumLoops() {
        return 20;
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15;
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5;
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            30, // stallLimit The current limit in Amps at 0 RPM
            18, // freeLimit The current limit at free speed (~4500 RPM)
            250 // limitRpm RPM below this gets stallLimit, above scales to freeLimit
        };
    }
}

    public static class Vortex extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 60; // Vortex specific emergency stop current
    }

    @Override
    public double getOverHeatTemp() {
        return 90; // Vortex specific overheat temperature
    }

    @Override
    public int getNumLoops() {
        return 20; // Vortex specific number of loops
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15; // Vortex specific loops required to shut off
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 2.0; // Vortex specific cooldown duration
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            40, // stallLimit The current limit in Amps at 0 RPM
            25, // freeLimit The current limit at free speed (~4500 RPM)
            300 // limitRpm RPM below this gets stallLimit, above scales to freeLimit
        };
    }
}
  
    public static class REV_Defualt extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 40;
    }

    @Override
    public double getOverHeatTemp() {
        return 80;
    }

    @Override
    public int getNumLoops() {
        return 20;
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15;
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5;
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[]{
            25, // stallLimit
            15, // freeLimit
            400 // limitRpm
        };
    }
}

    public class REV_Custom extends Motor_Type { 
    // Instance variables to store the motor parameters
    private final double emergencyStopCurrent;
    private final double overHeatTemp;
    private final int numLoops;
    private final int requiredLoopsToShutOff;
    private final double emergencyStopCooldownDuration;
    private final int[] ampLimits;

    // Constructor to accept custom values for the motor
    public REV_Custom(double emergencyStopCurrent,
                      double overHeatTemp,
                      int numLoops,
                      int requiredLoopsToShutOff,
                      double emergencyStopCooldownDuration,
                      int stallLimit,
                      int freeLimit,
                      int limitRpm) {
        this.emergencyStopCurrent = emergencyStopCurrent;
        this.overHeatTemp = overHeatTemp;
        this.numLoops = numLoops;
        this.requiredLoopsToShutOff = requiredLoopsToShutOff;
        this.emergencyStopCooldownDuration = emergencyStopCooldownDuration;
        
        // Setting the AMP limits (stallLimit, freeLimit, limitRpm)
        this.ampLimits = new int[] { stallLimit, freeLimit, limitRpm };
    }

    @Override
    public double getEmergencyStopCurrent() {
        return emergencyStopCurrent; // Return the custom value passed into the constructor
    }

    @Override
    public double getOverHeatTemp() {
        return overHeatTemp; // Return the custom value passed into the constructor
    }

    @Override
    public int getNumLoops() {
        return numLoops; // Return the custom value passed into the constructor
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return requiredLoopsToShutOff; // Return the custom value passed into the constructor
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return emergencyStopCooldownDuration; // Return the custom value passed into the constructor
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return ampLimits; // Return the custom amp limits
    }
}
    // </editor-fold>


    // === CTRE ===
  // <editor-fold>
    public static class Falcon extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 40; // Falcon 500 typical continuous stall current limit
    }

    @Override
    public double getOverHeatTemp() {
        return 85; // Falcon max safe temp
    }

    @Override
    public int getNumLoops() {
        return 20; // Number of loops for trip detection
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15; // Loops required to shut off after trip
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5; // Cooldown duration in seconds
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            35,  // SupplyCurrentLimit (amps)
            1,   // SupplyCurrentLowerTime (seconds)
            25,  // SupplyCurrentLowerLimit (amps)
            60   // StatorCurrentLimit (amps)
        };
    }
}

    public static class Kraken44 extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 44; // Slightly less than Kraken60 but higher than Falcon
    }

    @Override
    public double getOverHeatTemp() {
        return 95; // Moderate thermal tolerance
    }

    @Override
    public int getNumLoops() {
        return 22; // Number of loops to check before emergency stop
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 16; // Loops required to shut off after trip
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.75; // Cooldown seconds after emergency stop
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            40,  // SupplyCurrentLimit (amps)
            1,   // SupplyCurrentLowerTime (seconds)
            30,  // SupplyCurrentLowerLimit (amps)
            80   // StatorCurrentLimit (amps)
        };
    }
}

    public static class Kraken60 extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 60; // Kraken60 has higher rated current
    }

    @Override
    public double getOverHeatTemp() {
        return 100; // Example value, adjust based on motor spec
    }

    @Override
    public int getNumLoops() {
        return 25; // Slightly more forgiving window for Kraken
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 18; // Slightly more persistent threshold
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 2.0; // Longer cooldown for a high-performance motor
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            40,  // SupplyCurrentLimit: max current before limiting (amps)
            1,   // SupplyCurrentLowerTime: time before relaxing (seconds)
            35,  // SupplyCurrentLowerLimit: relaxed current (amps)
            80   // StatorCurrentLimit: peak motor current (amps)
        };
    }
}

    public static class CTRE_Defualt extends Motor_Type {

    @Override
    public double getEmergencyStopCurrent() {
        return 40; // rated current
    }

    @Override
    public double getOverHeatTemp() {
        return 85; // Example value, adjust based on motor spec
    }

    @Override
    public int getNumLoops() {
        return 20; // Number of loops for trip detection
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return 15; // Loops required to shut off after trip
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return 1.5; // Cooldown duration in seconds
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return new int[] {
            35,  // SupplyCurrentLimit (amps)
            1,   // SupplyCurrentLowerTime (seconds)
            25,  // SupplyCurrentLowerLimit (amps)
            60   // StatorCurrentLimit (amps)
        };
    }
}

    public class CTRE_Custom extends Motor_Type {
    // Instance variables to store the motor parameters
    private final double emergencyStopCurrent;
    private final double overHeatTemp;
    private final int numLoops;
    private final int requiredLoopsToShutOff;
    private final double emergencyStopCooldownDuration;
    private final int[] ampLimits;

    // Constructor to accept custom values for the motor
    public CTRE_Custom(double emergencyStopCurrent,
                        double overHeatTemp,
                        int numLoops,
                        int requiredLoopsToShutOff,
                        double emergencyStopCooldownDuration,
                        int supplyCurrentLimit,
                        int supplyCurrentLowerTime,
                        int supplyCurrentLowerLimit,
                        int statorCurrentLimit) {
        this.emergencyStopCurrent = emergencyStopCurrent;
        this.overHeatTemp = overHeatTemp;
        this.numLoops = numLoops;
        this.requiredLoopsToShutOff = requiredLoopsToShutOff;
        this.emergencyStopCooldownDuration = emergencyStopCooldownDuration;
        
        // Setting the AMP limits
        this.ampLimits = new int[] { 
            supplyCurrentLimit, 
            supplyCurrentLowerTime, 
            supplyCurrentLowerLimit, 
            statorCurrentLimit 
        };
    }

    @Override
    public double getEmergencyStopCurrent() {
        return emergencyStopCurrent; // Return the custom value passed into the constructor
    }

    @Override
    public double getOverHeatTemp() {
        return overHeatTemp; // Return the custom value passed into the constructor
    }

    @Override
    public int getNumLoops() {
        return numLoops; // Return the custom value passed into the constructor
    }

    @Override
    public int getRequiredLoopsToShutOff() {
        return requiredLoopsToShutOff; // Return the custom value passed into the constructor
    }

    @Override
    public double getEmergencyStopCooldownDuration() {
        return emergencyStopCooldownDuration; // Return the custom value passed into the constructor
    }

    @Override
    public int[] getDefaultAmpLimits() {
        return ampLimits; // Return the custom amp limits
    }
}
    // </editor-fold>
}