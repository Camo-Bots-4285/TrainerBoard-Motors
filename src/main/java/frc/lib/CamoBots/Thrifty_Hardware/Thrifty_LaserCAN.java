package frc.lib.CamoBots.Thrifty_Hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

/**
 * Thrifty_LaserCAN
 * 
 * This subsystem handles setup and data collection from the ThriftyBot LaserCAN distance sensor.
 * The sensor uses a CAN interface and supports different ranging modes and region of interest settings.
 * 
 * Features:
 * - Configurable short or long range mode
 * - Adjustable observation timing budget
 * - User-defined region of interest (ROI)
 * - Simple distance (in cm) and blocked detection support
 * 
 * @author CD
 * @since 2025 FRC Season
 */
public class Thrifty_LaserCAN extends SubsystemBase {

    // ========== Hardware & Settings ==========

    private LaserCan m_LaserCAN;
    private LaserCan.RangingMode Range;
    private double Blocked_Distance;
    private LaserCan.TimingBudget ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_33MS;

    /**
     * Creates a new LaserCAN sensor subsystem.
     *
     * @param CAN_ID                  CAN ID of the LaserCAN device
     * @param ShortRange              True for short range (1.3m max), false for long range (4m max)
     * @param Observation_Area       Array of 4 values defining ROI: [centerX, centerY, width, height]
     * @param ObservationTimingBudget Timing budget index (0 = 20ms, 1 = 33ms, 2 = 50ms, 3 = 100ms)
     * @param Blocked_Distance       Distance in cm to determine if the path is "blocked"
     */
    public Thrifty_LaserCAN(
        int CAN_ID,
        Boolean ShortRange,
        int[] Observation_Area,
        int ObservationTimingBudget,
        double Blocked_Distance
    ) {
        // Required for CANBridge mode (if using simulation or certain hardware)
        CanBridge.runTCP();

        // Save the blocked distance threshold
        this.Blocked_Distance = Blocked_Distance;

        // Select range mode (short or long)
        Range = ShortRange ? LaserCan.RangingMode.SHORT : LaserCan.RangingMode.LONG;

        // Select timing budget based on parameter
        switch (ObservationTimingBudget) {
            case 0: ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_20MS; break;
            case 1: ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_33MS; break;
            case 2: ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_50MS; break;
            case 3: ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_100MS; break;
            default: ObservationTimingBudgetValue = LaserCan.TimingBudget.TIMING_BUDGET_33MS; break;
        }

        // Initialize LaserCAN device on specified CAN ID
        m_LaserCAN = new LaserCan(CAN_ID);

        // Optional configuration - only needed if GrappleHook wasn't used
        try {
            // Set range mode
            m_LaserCAN.setRangingMode(Range);

            // Set region of interest (x, y, width, height)
            m_LaserCAN.setRegionOfInterest(
                new LaserCan.RegionOfInterest(
                    Observation_Area[0], Observation_Area[1],
                    Observation_Area[2], Observation_Area[3]
                )
            );

            // Set timing budget (accuracy vs. speed)
            m_LaserCAN.setTimingBudget(ObservationTimingBudgetValue);

        } catch (ConfigurationFailedException e) {
            // Configuration failed – skip to avoid crashing robot
            // System.out.println("LaserCAN configuration failed: " + e.getMessage());
        }
    }

    // ========== Public Methods ==========

    /**
     * Gets the current distance reading from the LaserCAN sensor.
     *
     * @return Distance in centimeters
     */
    public double getDistance_cm() {
        LaserCan.Measurement measurement1 = m_LaserCAN.getMeasurement();
        return measurement1.distance_mm / 10.0; // Convert from mm to cm
    }

    /**
     * Checks if the measured distance is less than the blocked threshold.
     *
     * @return True if something is within the blocked distance
     */
    public boolean getBlocked() {
        return getDistance_cm() <= Blocked_Distance;
    }
}
