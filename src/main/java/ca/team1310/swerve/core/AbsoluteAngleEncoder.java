package ca.team1310.swerve.core;

/**
 * Interface for an absolute angle encoder. It exposes the methods that are necessary for an angle
 * motor.
 */
public interface AbsoluteAngleEncoder {
  /**
   * Get the absolute location of the encoder.
   *
   * @return The absolute location of the encoder in degrees, from 0 to 360. Returns -1 on error.
   */
  double getPosition();

  /**
   * Get the cached absolute location of the encoder without performing expensive refresh or health
   * check operations. The cached value comes from the StatusSignal's auto-refresh (typically
   * 100Hz), so it is at most 10ms old.
   *
   * <p>This is suitable for high-frequency reads such as odometry updates where latency of a full
   * refresh is unacceptable.
   *
   * @return The absolute location of the encoder in degrees, from 0 to 360. Returns -1 on error.
   */
  default double getPositionCached() {
    return getPosition();
  }

  /**
   * Are there any active faults on this motor
   *
   * @return true if there are active faults
   */
  boolean hasFaults();
}
