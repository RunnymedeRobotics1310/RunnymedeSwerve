package ca.team1310.swerve.math;

/**
 * TEMPORARY class - for internal use only by SwerveMath. DO NOT USE OUTSIDE THIS EXPERIMENT
 *
 * @author Tony Field
 * @since 2025-05-14 07:25
 */
class RRRotation2d {

  final double radians;
  final double cos;
  final double sin;

  RRRotation2d(double radians) {
    this.radians = radians;
    cos = Math.cos(radians);
    sin = Math.sin(radians);
  }

  RRRotation2d(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude > 1e-6) {
      cos = x / magnitude;
      sin = y / magnitude;
    } else {
      cos = 1.0;
      sin = 0.0;
    }
    radians = Math.atan2(sin, cos);
  }

  RRRotation2d unaryMinus() {
    return new RRRotation2d(-radians);
  }

  RRRotation2d rotate(RRRotation2d other) {
    return new RRRotation2d(cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos);
  }

  RRRotation2d minus(RRRotation2d other) {
    return rotate(other.unaryMinus());
  }

  RRRotation2d plus(RRRotation2d other) {
    return rotate(other);
  }
}
