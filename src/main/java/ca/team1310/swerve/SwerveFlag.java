package ca.team1310.swerve;

/**
 * Runtime configuration flags for the swerve drive that can be toggled during operation. Use with
 * {@link RunnymedeSwerveDrive#setFlag(SwerveFlag, boolean)} and {@link
 * RunnymedeSwerveDrive#getFlag(SwerveFlag)}.
 */
public enum SwerveFlag {

  /**
   * Controls whether vision measurements are incorporated into the pose estimator. Disable during
   * autonomous routines where vision updates may cause unwanted pose corrections. Enabled by
   * default.
   */
  VISION_ODOMETRY_ENABLED;
}
