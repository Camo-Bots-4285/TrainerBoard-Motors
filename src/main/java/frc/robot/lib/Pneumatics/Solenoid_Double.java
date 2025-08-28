package frc.robot.lib.Pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Solenoid_Double
 * 
 * This class wraps a DoubleSolenoid object for use in pneumatics-controlled mechanisms.
 * It provides an easy interface to toggle between extended and retracted states.
 * 
 * Features:
 * - Set solenoid to extend (forward) or retract (reverse)
 * 
 * Hardware:
 * - Designed for use with the REV Pneumatics Hub (PH)
 * - Requires two pneumatic channels (one for forward, one for reverse)
 * 
 * Reference: 
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/solenoids.html
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class Solenoid_Double extends SubsystemBase {

  // Internal DoubleSolenoid object that controls the pneumatic output
  private final DoubleSolenoid m_doubleSolenoid;

  /**
   * Creates a new DoubleSolenoid on the specified channels.
   * 
   * @param pneumatic_Hub         CAN ID of the REV Pneumatics Hub
   * @param pneumatic_Channel_On  Channel used to extend the solenoid
   * @param pneumatic_Channel_Off Channel used to retract the solenoid
   */
  public Solenoid_Double(int pneumatic_Hub, int pneumatic_Channel_On, int pneumatic_Channel_Off) {
    m_doubleSolenoid = new DoubleSolenoid(
      pneumatic_Hub,
      PneumaticsModuleType.REVPH,
      pneumatic_Channel_On,
      pneumatic_Channel_Off
    );
  }

  /**
   * Sets the solenoid to extend (true) or retract (false).
   * 
   * @param extended true to extend (kForward), false to retract (kReverse)
   */
  public void setSoleniodPosition(boolean extended) {
    if (extended) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

}

