package ca.team1310.swerve.vision;

/**
 * Callback interface for getting pose estimates from a vision system.
 *
 * @author Tony Field
 * @since 2025-02-15 12:39
 */
public interface VisionPoseCallback {
    /**
     * Get the current pose estimate from the vision system. If no estimate is available, return null.
     * <p>
     * This method will be called at a regular interval by the swerve drive. Note that the period is
     * not necessarily the robot period, and may be faster or slower depending on the swerve drive
     * configuration / common code.
     * <p>
     * The returned pose estimate will be set into the swerve drive's pose estimator.
     *
     * @param yaw     the current robot heading in degrees, ccw positive
     * @param yawRate the current rate of change of the robot heading in degrees per second
     * @return the current pose estimate, or null if no estimate is available.  The default implementation returns null.
     */
    default PoseEstimate getPoseEstimate(double yaw, double yawRate) {
        return null;
    }
}
