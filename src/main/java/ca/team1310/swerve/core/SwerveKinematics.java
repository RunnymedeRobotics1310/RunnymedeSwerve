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
  private final double maxOmegaRadPerSec;

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
   * @param maxOmegaRadPerSec the maximum achievable angular velocity of a module, in radians per
   *     second
   */
  SwerveKinematics(
      double wheelBase, double trackWidth, double maxSpeedMps, double maxOmegaRadPerSec) {
    double frameDiagonal = Math.hypot(wheelBase, trackWidth);
    this.wheelBaseOverFrameDiagonal = wheelBase / frameDiagonal;
    this.trackWidthOverFrameDiagonal = trackWidth / frameDiagonal;
    this.maxSpeedMps = maxSpeedMps;
    this.maxOmegaRadPerSec = maxOmegaRadPerSec;
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
   * @param x desired forward velocity, in m/s (forward is positive)
   * @param y desired sideways velocity from in m/s (left is positive)
   * @param w desired angular velocity from in rad/s, (counter-clockwise is positive)
   * @param dt robot period in seconds (used for discretize)
   */
  void calculateAndStoreModuleVelocities(double x, double y, double w, double dt) {
    double[] scaledSpeeds = {x, y, w};

    // correction #2: desaturate wheel speeds
    double sf =
        SwerveMath.getScaleFactor(
            wheelBaseOverFrameDiagonal,
            trackWidthOverFrameDiagonal,
            x / maxSpeedMps,
            y / maxSpeedMps,
            w / maxOmegaRadPerSec);
    if (sf > 1) {
      scaledSpeeds[0] /= sf;
      scaledSpeeds[1] /= sf;
      scaledSpeeds[2] /= sf;
      System.out.println("sf: " + sf);
    }

    // discretize after desaturating speeds
    double[] discretized =
        SwerveMath.discretize(scaledSpeeds[0], scaledSpeeds[1], scaledSpeeds[2], dt);

    // if needed, desaturate and discretize again
    double sf2 =
        SwerveMath.getScaleFactor(
            wheelBaseOverFrameDiagonal,
            trackWidthOverFrameDiagonal,
            discretized[0] / maxSpeedMps,
            discretized[1] / maxSpeedMps,
            discretized[2] / maxOmegaRadPerSec);
    if (sf2 > 1) {
      scaledSpeeds[0] /= sf2;
      scaledSpeeds[1] /= sf2;
      scaledSpeeds[2] /= sf2;
      discretized = SwerveMath.discretize(scaledSpeeds[0], scaledSpeeds[1], scaledSpeeds[2], dt);
      System.out.println("sf2: " + sf2);
    }

    var result =
        SwerveMath.calculateModuleVelocitiesOpt(
            wheelBaseOverFrameDiagonal,
            trackWidthOverFrameDiagonal,
            discretized[0] / maxSpeedMps,
            discretized[1] / maxSpeedMps,
            discretized[2] / maxOmegaRadPerSec);

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
   * @param frs front right module speed (metres per second)
   * @param fra front right module angle (radians)
   * @param fls front left module speed (metres per second)
   * @param fla front left module angle (radians)
   * @param bls back left module speed (metres per second)
   * @param bla back left module angle (radians)
   * @param brs back right module speed (metres per second)
   * @param bra back right module angle (radians)
   * @return an array containing the x, y, and omega components of the robot's velocity
   */
  public double[] calculateRobotVelocity(
      double frs,
      double fra,
      double fls,
      double fla,
      double bls,
      double bla,
      double brs,
      double bra) {
    return SwerveMath.calculateRobotVelocityOpt(
        wheelBaseOverFrameDiagonal,
        trackWidthOverFrameDiagonal,
        frs,
        fra,
        fls,
        fla,
        bls,
        bla,
        brs,
        bra);
  }
}
