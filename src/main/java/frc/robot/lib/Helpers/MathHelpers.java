package frc.robot.lib.Helpers;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * MathHelpers
 * 
 * This utility class provides commonly used math functions for robot programming.
 * It includes angle wrapping, deadband logic, clamping values, interpolation,
 * distance calculations, and more.
 * 
 * These helpers are designed to make robot control code simpler and more readable.
 * 
 * @author CD inspired by 341
 * @since 2025 FRC Season
 */
public class MathHelpers {

  /**
   * Checks if a value is within a given inclusive range.
   * 
   * @param value The value to check
   * @param minValue The minimum limit (inclusive)
   * @param maxValue The maximum limit (inclusive)
   * @return true if the value is within the range, false otherwise
   */
  public static boolean isInRange(double value, double minValue, double maxValue) {
    return (value >= minValue) && (value <= maxValue);
  }

  /**
   * Calculates the straight-line distance between two points (2D).
   * 
   * @param x1 X coordinate of the first point
   * @param y1 Y coordinate of the first point
   * @param x2 X coordinate of the second point
   * @param y2 Y coordinate of the second point
   * @return The distance between the two points
   */
  public static double getDistance(double x1, double y1, double x2, double y2) {
    return Math.hypot(x2 - x1, y2 - y1);
  }

  /**
   * Calculates the angle between two points from a third "vertex" point using vectors.
   * Uses the dot product and Law of Cosines.
   * 
   * @param x1 Point 1 X
   * @param y1 Point 1 Y
   * @param x2 Point 2 X
   * @param y2 Point 2 Y
   * @param x3 Vertex X (angle is at this point)
   * @param y3 Vertex Y
   * @return The angle in radians at point (x3, y3), in range [0, π]
   */
  public static double getAngleAtPointRadians(double x1, double y1, double x2, double y2, double x3, double y3) {
    double v1x = x1 - x3;
    double v1y = y1 - y3;
    double v2x = x2 - x3;
    double v2y = y2 - y3;

    double dot = v1x * v2x + v1y * v2y;
    double magV1 = Math.hypot(v1x, v1y);
    double magV2 = Math.hypot(v2x, v2y);

    if (magV1 == 0 || magV2 == 0) return 0;

    return Math.acos(dot / (magV1 * magV2));
  }

  /**
   * Calculates the smallest angular difference between two angles (in radians).
   * 
   * @param from Starting angle in radians
   * @param to Target angle in radians
   * @return Smallest difference in radians, in range [-π, π]
   */
  public static double getDifferenceInAngleRadians(double from, double to) {
    return boundAngleNegPiToPiRadians(to - from);
  }

  /**
   * Calculates the smallest angular difference between two angles (in degrees).
   * 
   * @param from Starting angle in degrees
   * @param to Target angle in degrees
   * @return Smallest difference in degrees, in range [-180°, 180°]
   */
  public static double getDifferenceInAngleDegrees(double from, double to) {
    return boundAngleNeg180to180Degrees(to - from);
  }

  /**
   * Wraps an angle in degrees to the range [0, 360).
   */
  public static double boundAngle0to360Degrees(double angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
  }

  /**
   * Wraps an angle in degrees to the range [-180, 180).
   */
  public static double boundAngleNeg180to180Degrees(double angle) {
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
  }

  /**
   * Wraps an angle in radians to the range [0, 2π).
   */
  public static double boundAngle0to2PiRadians(double angle) {
    while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
    while (angle < 0.0) angle += 2 * Math.PI;
    return angle;
  }

  /**
   * Wraps an angle in radians to the range [-π, π).
   */
  public static double boundAngleNegPiToPiRadians(double angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
  }

  /**
   * Applies a deadband to an input value.
   * Small values within the deadband are returned as 0.
   * 
   * @param value Input value
   * @param deadband Threshold value (usually small, like 0.05)
   * @return Zero if input is within deadband, otherwise the input value
   */
  public static double applyDeadband(double value, double deadband) {
    return (Math.abs(value) < deadband) ? 0.0 : value;
  }

  /**
   * Constrains a value between a minimum and maximum.
   * 
   * @param val Value to clamp
   * @param min Minimum value
   * @param max Maximum value
   * @return Value clamped between min and max
   */
  public static double minmax(double val, double min, double max) {
    return Math.min(Math.max(val, min), max);
  }

  /**
   * Alias for minmax — a more readable and common name.
   */
  public static double clamp(double val, double min, double max) {
    return minmax(val, min, max);
  }

  /**
   * Linearly interpolates between two numbers.
   * 
   * @param a Start value
   * @param b End value
   * @param t Interpolation amount (0 = a, 1 = b)
   * @return The interpolated result
   */
  public static double lerp(double a, double b, double t) {
    return a + t * (b - a);
  }

  /**
   * Calculates the 2D magnitude of a Pose2d's translation component.
   * 
   * @param pose A Pose2d (position + rotation)
   * @return Distance from (0, 0) to the pose's X,Y point
   */
  public static double pose2dMagnitude(Pose2d pose) {
    return Math.hypot(pose.getX(), pose.getY());
  }
}
