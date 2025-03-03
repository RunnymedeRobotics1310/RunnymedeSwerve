package ca.team1310.swerve.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Utility functions for swerve drive. */
public final class SwerveUtils {

  private SwerveUtils() {}

  /**
   * Ensure that rotation error is between -pi and pi radians.
   *
   * @param radians the rotation to normalize
   * @return the normalized rotation
   */
  public static double normalizeRotation(double radians) {
    radians = radians % (2 * Math.PI);

    if (radians > Math.PI) {
      radians -= (2 * Math.PI);
    } else if (radians < -Math.PI) {
      radians += (2 * Math.PI);
    }

    return radians;
  }

  /**
   * Normalize the degrees measurement to between -180 and 180
   *
   * @param degrees input degrees any size
   * @return a value between -180 and 180
   */
  public static double normalizeDegrees(double degrees) {
    // reduce the angle
    degrees = degrees % 360;

    // force it to be the positive remainder, so that 0 <= angle < 360
    degrees = (degrees + 360) % 360;

    // force into the minimum absolute value residue class, so that -180 < angle <= 180
    if (degrees > 180) degrees -= 360;
    return degrees;
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
   * <p>Note that the tolerance needs to be realistically achievable.
   *
   * @param measured the current heading of the robot from the pose
   * @param target the heading you would like to face
   * @param tolerance - the tolerance to use for the comparison
   * @return true if the robot is within the tolerance of the desired heading
   * @throws IllegalArgumentException if no tolerance is specified
   */
  public static boolean isCloseEnough(double measured, double target, double tolerance) {
    double delta = target - measured;
    return Math.abs(delta) <= tolerance;
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
}
