package frc.robot.lib.Pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * Compresor
 * 
 * This class controls the robot's compressor for pneumatic systems.
 * It uses the REV Pneumatics Hub (PH) to manage pressure automatically using analog sensing.
 * 
 * Features:
 * - Starts and stops the compressor in closed-loop (automatic) mode.
 * - Gets the current pressure reading from the PH sensor.
 * 
 * Notes:
 * - This example assumes you're using the REV PH module (not the older PCM).
 * - For this to work correctly, make sure an analog pressure sensor is wired to the PH.
 * 
 * Reference: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pressure.html
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class Compresor {

  // Compressor instance shared across the robot
  public static Compressor m_compressor;

  /**
   * Creates a new Compresor object.
   * 
   * @param Compressor_Channel The CAN ID of the REV Pneumatics Hub (PH)
   */
  public Compresor(int Compressor_Channel) {
    // Initialize the compressor with the given channel and set to use the REV PH
    m_compressor = new Compressor(
      Compressor_Channel,
      PneumaticsModuleType.REVPH
    );
  }

  /**
   * Starts the compressor in analog (automatic) mode.
   * It will turn on when pressure drops below 102.5 PSI,
   * and turn off when it reaches 115 PSI.
   */
  public void runCompressor() {
    m_compressor.enableAnalog(102.5, 115.0);
  }

  /**
   * Disables the compressor completely.
   * Useful for debugging or safety shutoff.
   */
  public void disableCompressor() {
    m_compressor.disable();
  }

  /**
   * Returns the current system pressure (PSI) from the analog sensor.
   * 
   * @return Current pressure reading in PSI
   */
  public double getCurrentPressure() {
    return m_compressor.getPressure();
  }
}