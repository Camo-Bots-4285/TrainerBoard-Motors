package frc.robot.lib.Cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * DriveCamera
 * 
 * Handles initialization of a USB camera for driver viewing or vision assistance.
 * Currently supports one camera stream but is designed to be extendable for 
 * multiple camera switching or dynamic stream selection in the future.
 * 
 * Example use case:
 * - One front-facing camera for intake alignment.
 * - One rear-facing camera for game piece placement.
 * - Logic can switch between them depending on robot state.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class DriveCamera {

  // Reference to the USB camera
  private UsbCamera m_Camera;

  /**
   * Creates a DriveCamera instance and starts automatic camera capture.
   * 
   * @param cameraID   The USB port ID of the camera.
   * @param frameRate  The desired frames per second (e.g., 15 or 30).
   * @param widthPixels  The width resolution in pixels (e.g., 320, 640).
   * @param heightPixels The height resolution in pixels (e.g., 240, 480).
   */
  public DriveCamera(int cameraID, int frameRate, int widthPixels, int heightPixels) {
    // Start automatic capture on the specified USB camera ID
    m_Camera = CameraServer.startAutomaticCapture(cameraID);

    // Apply frame rate and resolution settings
    m_Camera.setFPS(frameRate);
    m_Camera.setResolution(widthPixels, heightPixels);
  }

  /**
   * Returns the initialized USB camera.
   * 
   * @return UsbCamera instance being streamed.
   */
  public UsbCamera getCamera() {
    return m_Camera;
  }
}
