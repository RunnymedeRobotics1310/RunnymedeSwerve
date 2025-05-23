package ca.team1310.swerve.math;

/**
 * TEMPORARY class - for internal use only by SwerveMath.
 *
 * @author Tony Field
 * @since 2025-05-14 07:51
 */
class RRTwist2d {
  final double dx, dy, dtheta;

  RRTwist2d(double dx, double dy, double dtheta) {
    this.dx = dx;
    this.dy = dy;
    this.dtheta = dtheta;
  }
}
