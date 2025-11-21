package frc.lib.CamoBots.SelfDriving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * DriverLock
 * 
 * This class provides pose-based self-driving logic for autonomous and assisted driving.
 * It uses PID controllers and slew limiters to move the robot smoothly to a target X/Y/rotation position.
 * 
 * Features:
 * - PID control for translation and rotation
 * - Acceleration limiting via SlewRateLimiter
 * - Alliance-aware field-relative pose calculation
 * - Control toggles for individual axes (X, Y, Rotation)
 * 
 * Use DriveCalculationPose() in your command to generate the final X/Y/Rotation speeds toward a target.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class DriverLock {

  // ========== Pose Differences ==========
  public static double PoseDifferenceX;
  public static double PoseDifferenceY;
  public static double PoseDifferenceRotation;
  public static double PoseDifferenceRotationRaw;

  // ========== Raw Speeds Before Limiting ==========
  public static double XSpeed;
  public static double YSpeed;
  public static double RotationSpeed;

  // ========== Final Output Speeds ==========
  public static double XSpeedFinal;
  public static double YSpeedFinal;
  public static double RotationSpeedFinal;

  // ========== Completion Checks ==========
  public static boolean CloseEnough;
  public static double HowFar;
  public static double HowStraght;
  public static boolean TranslationClose;
  public static boolean RotationClose;

  // ========== Tuning Parameters ==========
  public static double MaxTranslationSpeed = 0.25; // Max translational speed (m/s)
  public static double MaxRotationSpeed = Math.PI; // Max rotational speed (rad/s)
  public static double Y_X_Ratio = 4;
  public static double XCompensationValue = 1;

  // ========== Acceleration Limits ==========
  static SlewRateLimiter MaxTranslationAccelerationX = new SlewRateLimiter(0.55);
  static SlewRateLimiter MaxTranslationAccelerationY = new SlewRateLimiter(0.55);
  static SlewRateLimiter MaxRotationAcceleration = new SlewRateLimiter(2 * Math.PI);

  // ========== PID Controllers ==========
  public static PIDController MoveToPoseTranslation = new PIDController(0.3, 0.0, 0.0);
  public static PIDController MoveToPoseRotation = new PIDController(0.0125, 0, 0);

  // ========== Control Toggles ==========
  public static boolean AssumeControlX = false;
  public static boolean AssumeControlY = false;
  public static boolean AssumeControlRotation = false;
  public static boolean AssumeControlFeild = false;
  public static boolean FeildCentric = false;

  /**
   * Gets the current pose from telemetry (field-relative).
   *
   * @return Current Pose2d of the robot
   */
  public static Pose2d currentPose2d() {
    return new Pose2d(
      // new Translation2d(Telemetry.m_poseArray[0], Telemetry.m_poseArray[1]),
      // new Rotation2d(Units.degreesToRadians(Telemetry.m_poseArray[2]))
    );
  }

  /**
   * Enables or disables control over all axes (X, Y, Rotation).
   * 
   * @param state true to take full control; false to release
   */
  public static void AssumeTotalControl(boolean state) {
    AssumeControlX = state;
    AssumeControlY = state;
    AssumeControlRotation = state;
  }

  /**
   * Enables or disables X-axis control.
   */
  public static void AssumeControlX(boolean state) {
    AssumeControlX = state;
  }

  /**
   * Enables or disables Y-axis control.
   */
  public void AssumeControlY(boolean state) {
    AssumeControlY = state;
  }

  /**
   * Enables or disables rotational control.
   */
  public static void AssumeControlRotation(boolean state) {
    AssumeControlRotation = state;
  }

  /**
   * Computes drive speeds required to move the robot to a target pose.
   * This should be called continuously during an autonomous or assisted movement.
   *
   * @param TargetPoseX        X target in meters
   * @param TargetPoseY        Y target in meters
   * @param TargetPoseRotation Target heading in degrees
   */
  public static void DriveCalculationPose(double TargetPoseX, double TargetPoseY, double TargetPoseRotation) {
    
    // Alliance-aware X/Y/Rotation difference calculation
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      PoseDifferenceX = TargetPoseX - currentPose2d().getX();
      PoseDifferenceY = TargetPoseY - currentPose2d().getY();
      PoseDifferenceRotationRaw = currentPose2d().getRotation().getDegrees() - TargetPoseRotation;
    } else {
      PoseDifferenceX = TargetPoseX - currentPose2d().getX();
      PoseDifferenceY = TargetPoseY - currentPose2d().getY();
      PoseDifferenceRotationRaw = currentPose2d().getRotation().getDegrees() - TargetPoseRotation;
    }

    // Normalize rotation to shortest path [-180, 180]
    if (PoseDifferenceRotationRaw > 180) {
      PoseDifferenceRotation = PoseDifferenceRotationRaw - 360;
    } else {
      PoseDifferenceRotation = PoseDifferenceRotationRaw;
    }

    // Apply PID output (drive toward pose)
    XSpeed = MoveToPoseTranslation.calculate(PoseDifferenceX, 0.0);
    YSpeed = MoveToPoseTranslation.calculate(PoseDifferenceY, 0.0);
    RotationSpeed = MoveToPoseRotation.calculate(PoseDifferenceRotation, 0.0);

    // Clamp max speed outputs
    XSpeed = Math.abs(XSpeed) < MaxTranslationSpeed ? XSpeed : MaxTranslationSpeed;
    YSpeed = Math.abs(YSpeed) < MaxTranslationSpeed ? YSpeed : MaxTranslationSpeed;
    RotationSpeed = Math.abs(RotationSpeed) < MaxRotationSpeed ? RotationSpeed : MaxRotationSpeed;

    // Apply acceleration limiters
    XSpeedFinal = MaxTranslationAccelerationX.calculate(XSpeed);
    YSpeedFinal = MaxTranslationAccelerationY.calculate(YSpeed);
    RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);

    // Check if we're close enough to the target to stop
    if (Math.abs(PoseDifferenceRotation) < 5.0 &&
        Math.abs(PoseDifferenceY) < 0.15 &&
        Math.abs(PoseDifferenceX) < 0.15) {
      CloseEnough = true;
    } else {
      CloseEnough = false;
    }
  }
}
