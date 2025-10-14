package ca.team1310.swerve.math;

/**
 * TEMPORARY class - for internal use only by SwerveMath. DO NOT USE OUTSIDE THIS EXPERIMENT
 *
 * @author Tony Field
 * @since 2025-05-14 07:27
 */
class RRPose2d {
  static final RRPose2d ZERO = new RRPose2d(0, 0, 0);
  final RRTranslation2d translation;
  final RRRotation2d rotation;

  RRPose2d(RRTranslation2d translation, RRRotation2d rotation) {
    this.translation = translation;
    this.rotation = rotation;
  }

  RRPose2d(double x, double y, double radians) {
    this.translation = new RRTranslation2d(x, y);
    this.rotation = new RRRotation2d(radians);
  }

  /**
   * Transforms the pose by the given transformation and returns the new pose. See + operator for
   * the matrix multiplication performed.
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  RRPose2d transformBy(RRTransform2d other) {
    return new RRPose2d(
        translation.plus(other.translation.rotate(rotation)), other.rotation.plus(rotation));
  }

  /**
   * Returns the current pose relative to the given pose.
   *
   * <p>This function can often be used for trajectory tracking or pose stabilization algorithms to
   * get the error between the reference and the current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that the current pose will
   *     be converted into.
   * @return The current pose relative to the new origin pose.
   */
  RRPose2d relativeTo(RRPose2d other) {
    var transform = new RRTransform2d(other, this);
    return new RRPose2d(transform.translation, transform.rotation);
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose.
   *
   * <pre>
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [  0,    0, 1][transform.t]
   * </pre>
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  RRPose2d plus(RRTransform2d other) {
    return transformBy(other);
  }

  /**
   * Obtain a new Pose2d from a (constant curvature) velocity.
   *
   * <p>See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
   * Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
   * derivation.
   *
   * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
   * update. When the user runs exp() on the previous known field-relative pose with the argument
   * being the twist, the user will receive the new field-relative pose.
   *
   * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
   * pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
   *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
   *     degrees since the previous pose update, the twist would be Twist2d(0.01, 0.0,
   *     Units.degreesToRadians(0.5)).
   * @return The new pose of the robot.
   */
  RRPose2d exp(RRTwist2d twist) {
    double dx = twist.dx;
    double dy = twist.dy;
    double dtheta = twist.dtheta;

    double sinTheta = Math.sin(dtheta);
    double cosTheta = Math.cos(dtheta);

    double s;
    double c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sinTheta / dtheta;
      c = (1 - cosTheta) / dtheta;
    }
    var transform =
        new RRTransform2d(
            new RRTranslation2d(dx * s - dy * c, dx * c + dy * s),
            new RRRotation2d(cosTheta, sinTheta));

    return this.plus(transform);
  }

  /**
   * Returns a Twist2d that maps this pose to the end pose.
   *
   * <p>If c is the output of {@code a.Log(b)}, then {@code a.Exp(c)} would yield b.
   *
   * @param end The end pose for the transformation.
   * @return The twist that maps this to end.
   */
  RRTwist2d log(RRPose2d end) {
    final RRPose2d transform = end.relativeTo(this);
    final double dtheta = transform.rotation.radians;
    final double halfDtheta = dtheta / 2.0;
    final double cosMinusOne = transform.rotation.cos - 1.0;
    final double halfThetaByTanOfHalfDtheta =
        (Math.abs(cosMinusOne) < 1E-9)
            ? 1.0 - 1.0 / 12.0 * dtheta * dtheta
            : -(halfDtheta * transform.rotation.sin) / cosMinusOne;
    final RRTranslation2d translationPart =
        transform
            .translation
            .rotate(new RRRotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta))
            .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));
    return new RRTwist2d(translationPart.x, translationPart.y, dtheta);
  }
}
