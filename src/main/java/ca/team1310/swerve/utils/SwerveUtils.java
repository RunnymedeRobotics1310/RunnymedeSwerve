package ca.team1310.swerve.utils;

import ca.team1310.swerve.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Utility functions for swerve drive. */
public final class SwerveUtils {

  private SwerveUtils() {}

  /**
   * Normalize the degrees measurement to between -180 and 180
   *
   * @param degrees input degrees any size
   * @return a value between -180 and 180
   */
  public static double normalizeDegrees(double degrees) {
    return SwerveMath.normalizeDegrees(degrees);
  }

  /**
   * Compute the difference between two poses. Note, as of 2024-02-18, Pose2d.minus(Pose2d) does
   * this incorrectly.
   *
   * @param target the desired pose
   * @param current the curren tpose
   * @return the difference between the two poses.
   */
  public static Transform2d difference(Pose2d target, Pose2d current) {
    Translation2d dx =
        new Translation2d(target.getX() - current.getX(), target.getY() - current.getY());
    Rotation2d dw = target.getRotation().minus(current.getRotation());
    return new Transform2d(dx, dw);
  }

  /**
   * Returns true when the robot heading is within <code>tolerance</code> of the desired location.
   *
   * @param measuredDegrees the current heading of the robot from the pose
   * @param targetDegrees the heading you would like to face
   * @param toleranceDegrees - the tolerance to use for the comparison
   * @return true if the robot is within the tolerance of the desired heading
   * @throws IllegalArgumentException if no tolerance is specified
   */
  public static boolean isCloseEnough(
      double measuredDegrees, double targetDegrees, double toleranceDegrees) {

    // normalize to positive value
    measuredDegrees = measuredDegrees % 360;
    measuredDegrees = (measuredDegrees + 360) % 360;
    targetDegrees = targetDegrees % 360;
    targetDegrees = (targetDegrees + 360) % 360;

    double delta = targetDegrees - measuredDegrees;
    delta = normalizeDegrees(delta);
    return Math.abs(delta) <= toleranceDegrees;
  }

  /**
   * Returns true when the robot is located within <code>toleranceMetres</code>> of the desired
   * location
   *
   * @param currentLocation the current location of the robot
   * @param desiredLocation the desired location of the robot
   * @param toleranceMetres the tolerance to use for the comparison
   * @return true if the robot is within the tolerance of the desired location
   */
  public static boolean isCloseEnough(
      Translation2d currentLocation, Translation2d desiredLocation, double toleranceMetres) {
    return Math.abs(currentLocation.getDistance(desiredLocation)) <= toleranceMetres;
  }

  /**
   * Constrains a value to lie between a specified minimum and maximum range.
   *
   * @param min the minimum allowable value
   * @param value the input value to be clamped
   * @param max the maximum allowable value
   * @return the clamped value, which will be within the range [min, max]
   */
  public static double clamp(double min, double value, double max) {
    return Math.max(min, Math.min(value, max));
  }
}
