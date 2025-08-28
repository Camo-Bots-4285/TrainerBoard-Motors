package frc.robot.lib.Pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Solenoid_Single
 * 
 * This class wraps a single-acting Solenoid for controlling simple pneumatic mechanisms,
 * such as pistons that only extend and rely on an internal spring or external force to retract.
 * 
 * Features:
 * - Set solenoid on/off manually
 * - Pulse the solenoid for a fixed duration (e.g., for one-shot movements)
 * 
 * Hardware:
 * - Designed for the REV Pneumatics Hub (PH)
 * - Uses a single pneumatic channel
 * 
 * Reference:
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/solenoids.html
 * 
 * @author YourName
 * @since 2025 FRC Season
 */
public class Solenoid_Single extends SubsystemBase {

  // Internal solenoid object tied to one pneumatic output
  private final Solenoid m_singleSolenoid;

  /**
   * Constructs a new Solenoid_Single instance.
   * 
   * @param pneumatic_Hub     CAN ID of the Pneumatics Hub
   * @param pneumatic_Channel Pneumatic channel used for this solenoid
   */
  public Solenoid_Single(int pneumatic_Hub, int pneumatic_Channel) {
    m_singleSolenoid = new Solenoid(
      pneumatic_Hub,
      PneumaticsModuleType.REVPH,
      pneumatic_Channel
    );
  }

  /**
   * Sets the solenoid state (true = on/extend, false = off/retract).
   * 
   * @param extended true to activate (extend), false to deactivate (retract)
   */
  public void setSoleniodPosition(boolean extended) {
    m_singleSolenoid.set(extended);
  }

  /**
   * Sends a timed pulse to the solenoid. The solenoid will activate for the specified duration.
   * Useful for one-shot actuations where you don't want to leave it energized.
   * 
   * @param Time_0_00x Duration of the pulse in seconds (e.g., 0.05 for 50ms)
   */
  public void setSoleniodPulse(double Time_0_00x) {
    m_singleSolenoid.setPulseDuration(Time_0_00x);
    m_singleSolenoid.startPulse();
  }
}
