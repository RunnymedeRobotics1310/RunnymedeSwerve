package ca.team1310.swerve.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Holder for vision pose information to be fed into the swerve drive.
 *
 * @author Tony Field
 * @since 2025-02-15 13:01
 */
public interface VisionPoseEstimate {
  /**
   * Get the robot pose as determined by vision.
   *
   * @return the current vision pose information. Never null
   */
  Pose2d getPose();

  /**
   * Get the timestamp of the vision pose information in seconds.
   *
   * @return the timestamp of the vision pose information in seconds.
   */
  double getTimestampSeconds();

  /**
   * Get the standard deviations of the vision pose information, if it has changed since last time
   * this was called. Note that this method should not return the same value twice in a row, as this
   * will result in a performance degradation of the estimator.
   *
   * @return matrix containing standard deviations, or null if no changes have occurred
   */
  Matrix<N3, N1> getStandardDeviations();
}
