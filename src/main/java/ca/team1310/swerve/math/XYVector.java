package ca.team1310.swerve.math;

/**
 * Mutable XY vector.
 *
 * @author Tony Field
 * @since 2025-05-14 07:53
 */
class XYVector {
  double x;
  double y;
  double magnitude;
  double angleRadians;

  XYVector(double x, double y) {
    this.x = x;
    this.y = y;
    magnitude = Math.hypot(x, y);
    angleRadians = Math.atan2(y, x);
  }

  void add(XYVector v) {
    x += v.x;
    y += v.y;
    angleRadians = Math.atan2(y, x);
    magnitude = Math.hypot(x, y);
  }

  void rotate(double radians) {
    angleRadians += radians;
    x = magnitude * Math.cos(angleRadians);
    y = magnitude * Math.sin(angleRadians);
  }

  void scale(double factor) {
    x *= factor;
    y *= factor;
    magnitude = Math.hypot(x, y);
  }
}
