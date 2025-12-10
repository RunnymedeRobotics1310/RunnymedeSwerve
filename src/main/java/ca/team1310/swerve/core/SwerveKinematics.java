package ca.team1310.swerve.core;

import ca.team1310.swerve.math.SwerveMath;

/**
 * @author Tony Field
 * @since 2025-05-13 16:40
 */
public class SwerveKinematics {

  private final double wheelBaseOverFrameDiagonal;
  private final double trackWidthOverFrameDiagonal;
  private final double maxSpeedMps;
  private final double maxOmegaDegPerSec;
  private final double discretizeTransScale;
  private final double discretizeRotScale;
  private final double discretizeNormalScale;

  private final ModuleDirective fr;
  private final ModuleDirective fl;
  private final ModuleDirective bl;
  private final ModuleDirective br;

  /**
   * Construct an instance of SwerveMath for a rectangular robot. Preliminary calculations will be
   * performed to allow for optimal performance when calculating individual module velocities.
   *
   * @param wheelBase the wheelbase of the drivetrain, (i.e. from the front to the back of the
   *     robot). Units are not relevant.
   * @param trackWidth the track width of the drivetrain (i.e. from the left to the right of the
   *     robot). Units are not relevant.
   * @param maxSpeedMps the maximum achievable speed of a module, in metres per second
   * @param maxOmegaDegPerSec the maximum achievable angular velocity of a module, in radians per
   *     second
   * @param discretizeTransScale The scale factor for the translation contribution to the normal
   *     vector
   * @param discretizeRotScale The scale factor for the rotation contribution to the normal vector
   * @param discretizeNormalScale An overall scale factor for the normal vector
   */
  SwerveKinematics(
      double wheelBase,
      double trackWidth,
      double maxSpeedMps,
      double maxOmegaDegPerSec,
      double discretizeTransScale,
      double discretizeRotScale,
      double discretizeNormalScale) {
    double frameDiagonal = Math.hypot(wheelBase, trackWidth);
    this.wheelBaseOverFrameDiagonal = wheelBase / frameDiagonal;
    this.trackWidthOverFrameDiagonal = trackWidth / frameDiagonal;
    this.maxSpeedMps = maxSpeedMps;
    this.maxOmegaDegPerSec = maxOmegaDegPerSec;
    this.discretizeTransScale = discretizeTransScale;
    this.discretizeRotScale = discretizeRotScale;
    this.discretizeNormalScale = discretizeNormalScale;
    this.fr = new ModuleDirective();
    this.fl = new ModuleDirective();
    this.bl = new ModuleDirective();
    this.br = new ModuleDirective();
  }

  /**
   * Calculate the module velocities for the swerve drive when operating in robot-oriented mode. The
   * individual wheel vectors are stored inside this instance where they can be retrieved using the
   * getter methods. The individual wheel vectors are returned in metres per second and degrees,
   * counter-clockwise positive.
   *
   * <p>Note that Correction 2 (desaturate wheel speeds) and Correction 4 (discretize) are both
   * applied when performing these calculations.
   *
   * @param x desired forward velocity, in power (-1.0 - 1.0) (forward is positive)
   * @param y desired sideways velocity in power (-1.0 - 1.0) (left is positive)
   * @param w desired angular velocity in power (-1.0 - 1.0), (counter-clockwise is positive)
   */
  void calculateModuleVelocities(double x, double y, double w) {
    double[] scaled = {x, y, w};

    // Correction #2: desaturate wheel speeds
    double sf =
        SwerveMath.computeVelocityScaleFactor(
            trackWidthOverFrameDiagonal, wheelBaseOverFrameDiagonal, x, y, w);
    if (sf > 1) {
      scaled[0] /= sf;
      scaled[1] /= sf;
      scaled[2] /= sf;
      //      System.out.println("sf1: " + sf);
    }

    // Correction #4 - discretize after desaturating speeds
    double[] discretized =
        SwerveMath.discretize(
            scaled[0],
            scaled[1],
            scaled[2],
            discretizeTransScale,
            discretizeRotScale,
            discretizeNormalScale);

    // After discretizing, we may have saturated wheel speeds again.
    // Check, and desaturate again if necessary
    double sf2 =
        SwerveMath.computeVelocityScaleFactor(
            trackWidthOverFrameDiagonal,
            wheelBaseOverFrameDiagonal,
            discretized[0],
            discretized[1],
            discretized[2]);
    if (sf2 > 1) {
      scaled[0] /= sf2;
      scaled[1] /= sf2;
      scaled[2] /= sf2;
      discretized =
          SwerveMath.discretize(
              scaled[0],
              scaled[1],
              scaled[2],
              discretizeTransScale,
              discretizeRotScale,
              discretizeNormalScale);
      //      System.out.println("sf2: " + sf2);
    }

    var result =
        SwerveMath.calculateModuleVelocities(
            trackWidthOverFrameDiagonal,
            wheelBaseOverFrameDiagonal,
            discretized[0],
            discretized[1],
            discretized[2]);

    // convert from -1.0 - 1.0 into to m/s
    result[0] *= maxSpeedMps;
    result[2] *= maxSpeedMps;
    result[4] *= maxSpeedMps;
    result[6] *= maxSpeedMps;

    // convert from radians (-PI to PI) to degrees (-180 to 180)
    result[1] = Math.toDegrees(result[1]);
    result[3] = Math.toDegrees(result[3]);
    result[5] = Math.toDegrees(result[5]);
    result[7] = Math.toDegrees(result[7]);

    // save module directives
    fr.set(result[0], result[1]);
    fl.set(result[2], result[3]);
    bl.set(result[4], result[5]);
    br.set(result[6], result[7]);
  }

  /**
   * Get the front right module velocity object
   *
   * @return module velocities in m/s and degrees, counter-clockwise positive
   */
  ModuleDirective getFrontRight() {
    return fr;
  }

  /**
   * Get the front left module velocity object
   *
   * @return module velocities in m/s and degrees, counter-clockwise positive
   */
  ModuleDirective getFrontLeft() {
    return fl;
  }

  /**
   * Get the back left module velocity object
   *
   * @return module velocities in m/s and degrees, counter-clockwise positive
   */
  ModuleDirective getBackLeft() {
    return bl;
  }

  /**
   * Get the back right module velocity object
   *
   * @return module velocities in m/s and degrees, counter-clockwise positive
   */
  ModuleDirective getBackRight() {
    return br;
  }

  /**
   * Compute the robot's velocity (vX (metres per second), vY (metres per second), omega (radians
   * per second)) given the velocities of the four swerve modules.
   *
   * @param frsMps front right module speed (metres per second)
   * @param fraDeg front right module angle (degrees)
   * @param flsMps front left module speed (metres per second)
   * @param flaDeg front left module angle (degrees)
   * @param blsMps back left module speed (metres per second)
   * @param blaDeg back left module angle (degrees)
   * @param brsMps back right module speed (metres per second)
   * @param braDeg back right module angle (degrees)
   * @return array containing vX (m/s), vY (m/s), and w (rad/s)
   */
  public double[] calculateRobotVelocity(
      double frsMps,
      double fraDeg,
      double flsMps,
      double flaDeg,
      double blsMps,
      double blaDeg,
      double brsMps,
      double braDeg) {
    // convert inputs

    double frsPwr = frsMps / maxSpeedMps;
    double flsPwr = flsMps / maxSpeedMps;
    double blsPwr = blsMps / maxSpeedMps;
    double brsPwr = brsMps / maxSpeedMps;
    double fraRad = Math.toRadians(fraDeg);
    double flaRad = Math.toRadians(flaDeg);
    double blaRad = Math.toRadians(blaDeg);
    double braRad = Math.toRadians(braDeg);
    // grab calculated output
    double[] velPwr =
        SwerveMath.calculateRobotVelocity(
            trackWidthOverFrameDiagonal,
            wheelBaseOverFrameDiagonal,
            frsPwr,
            fraRad,
            flsPwr,
            flaRad,
            blsPwr,
            blaRad,
            brsPwr,
            braRad);
    double xMps = velPwr[0] * maxSpeedMps;
    double yMps = velPwr[1] * maxSpeedMps;
    double omegaRadPerSec = velPwr[2] * Math.toRadians(maxOmegaDegPerSec);
    return new double[] {xMps, yMps, omegaRadPerSec};
  }
}
