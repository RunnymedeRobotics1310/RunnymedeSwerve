package ca.team1310.swerve.core;

/**
 * @author Tony Field
 * @since 2025-02-01 19:19
 */
public class ModuleDirective {

  private double speed = 0;
  private double angle = 0;

  /**
   * Set the angle and speed of the module
   *
   * @param speed in m/s
   * @param angle in degrees (ccw positive)
   */
  public void set(double speed, double angle) {
    this.speed = speed;
    this.angle = angle;
  }

  /**
   * Get the angle of the module
   *
   * @return the angle of the module in degrees from -180 to 180 (ccw positive)
   */
  public double getAngle() {
    return angle;
  }

  /**
   * Get the speed of the module
   *
   * @return the speed of the degrees in m/s
   */
  public double getSpeed() {
    return speed;
  }
}
