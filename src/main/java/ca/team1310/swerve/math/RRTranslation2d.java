package ca.team1310.swerve.math;

/**
 * TEMPORARY class - for internal use only by SwerveMath. DO NOT USE OUTSIDE THIS EXPERIMENT
 *
 * @author Tony Field
 * @since 2025-05-14 07:22
 */
class RRTranslation2d {
  final double x;
  final double y;

  RRTranslation2d(double x, double y) {
    this.x = x;
    this.y = y;
  }

  RRTranslation2d plus(RRTranslation2d other) {
    return new RRTranslation2d(x + other.x, y + other.y);
  }

  RRTranslation2d minus(RRTranslation2d other) {
    return new RRTranslation2d(x - other.x, y - other.y);
  }

  RRTranslation2d rotate(RRRotation2d other) {
    return new RRTranslation2d(x * other.cos - y * other.sin, x * other.sin + y * other.cos);
  }

  RRTranslation2d times(double scalar) {
    return new RRTranslation2d(x * scalar, y * scalar);
  }
}
