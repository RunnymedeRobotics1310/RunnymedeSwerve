/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.vision.config;

/**
 * Configuration for a Limelight vision system, including vision trust parameters for pose
 * estimation fusion.
 *
 * @param limelightName the name of the limelight to use
 * @param fieldExtentX the extent of the field in the X direction
 * @param fieldExtentY the extent of the field in the Y direction
 * @param baseStdDevXY base standard deviation for vision XY measurements in metres
 * @param baseStdDevTheta base standard deviation for vision heading measurements in radians
 * @param distanceScaleFactor scaling factor for std dev growth with distance squared
 * @param maxTrustDistanceMetres reject vision readings beyond this average tag distance
 * @param outlierRejectionThresholdMetres reject vision poses farther than this from current
 *     estimate
 * @param minTagCountForLowStdDev minimum tag count to get the best (1x) std dev; fewer tags get a
 *     2x penalty
 * @author Tony Field
 * @since 2025-09-23 16:16
 */
public record LimelightConfig(
    String limelightName,
    double fieldExtentX,
    double fieldExtentY,
    double baseStdDevXY,
    double baseStdDevTheta,
    double distanceScaleFactor,
    double maxTrustDistanceMetres,
    double outlierRejectionThresholdMetres,
    int minTagCountForLowStdDev) {

  /**
   * Backward-compatible constructor with default vision trust parameters.
   *
   * @param limelightName the name of the limelight to use
   * @param fieldExtentX the extent of the field in the X direction
   * @param fieldExtentY the extent of the field in the Y direction
   */
  public LimelightConfig(String limelightName, double fieldExtentX, double fieldExtentY) {
    this(limelightName, fieldExtentX, fieldExtentY, 0.4, 9999999, 0.08, 5.0, 1.0, 2);
  }
}
