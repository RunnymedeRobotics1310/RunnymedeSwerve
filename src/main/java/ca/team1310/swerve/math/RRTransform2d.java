package ca.team1310.swerve.math;

/**
 * TEMPORARY class - for internal use only by SwerveMath.
 *
 * @author Tony Field
 * @since 2025-05-14 07:52
 */
class RRTransform2d {
  final RRTranslation2d translation;
  final RRRotation2d rotation;

  RRTransform2d(RRTranslation2d translation, RRRotation2d rotation) {
    this.translation = translation;
    this.rotation = rotation;
  }

  RRTransform2d(RRPose2d initial, RRPose2d last) {
    translation = last.translation.minus(initial.translation).rotate(initial.rotation.unaryMinus());
    rotation = last.rotation.minus(initial.rotation);
  }
}
