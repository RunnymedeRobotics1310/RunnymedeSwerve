package ca.team1310.swerve.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.team1310.swerve.math.SwerveMath;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * @author Tony Field
 * @since 2025-02-19 21:14
 */
public class SwerveMathTest {

  /**
   * The epsilon to use for double comparisons. Values off by more than this amount are considered
   * different.
   */
  private static final double EPSILON = 1e-5;

  private static final double trackWidth = .58;
  private static final double wheelBase = .67;
  //  private static final double trackWidth = 16.75 * 0.0254;
  //  private static final double wheelBase = 16.75 * 0.0254;
  private static final double hypot = Math.hypot(wheelBase, trackWidth);
  private static final double wheelBaseOverFrameDiagonal = wheelBase / hypot;
  private static final double trackWidthOverFrameDiagonal = trackWidth / hypot;
  private static final double maxModuleSpeedMps = 5.3;
  private static final double maxOmegaRadPerSec =
      0.75 * 2 * Math.PI; // rotations per sec * 2pi (4.7123889804)
  private static final double transScale = 0.55;
  private static final double rotScale = 0.85;
  private static final double normalScale = 0.65;
  private static final double moduleUpdatePeriodSec = .005;
  private static final SwerveKinematics kin =
      new SwerveKinematics(
          wheelBase,
          trackWidth,
          maxModuleSpeedMps,
          maxOmegaRadPerSec,
          transScale,
          rotScale,
          normalScale);

  @Test
  public void test() {
    assertTrue(true, "true is true");
  }

  @ParameterizedTest
  @CsvSource({
    "0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0",
    "0,0,0,0,0,0,0,0,0,0,0",
    "1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0",
    "0, 1, 0, 1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268",
    "0, 1, 0, 1, 1.5707963267948966, 1, 1.5707963267948966, 1, 1.5707963267948966, 1, 1.5707963267948966",
    "0.7071067812, 0.7071067812, 0, 1, 0.7853981634, 1, 0.7853981634, 1, 0.7853981634, 1, 0.7853981634"
  })
  public void testToModuleVelocities(
      double x,
      double y,
      double omega,
      double expectedFrs,
      double expectedFra,
      double expectedFls,
      double expectedFla,
      double expectedBls,
      double expectedBla,
      double expectedBrs,
      double expectedBra) {
    double[] velocities =
        SwerveMath.calculateModuleVelocities(
            trackWidthOverFrameDiagonal, wheelBaseOverFrameDiagonal, x, y, omega);

    assertEquals(expectedFrs, velocities[0], EPSILON, "frs");
    assertEquals(expectedFra, velocities[1], EPSILON, "fra");
    assertEquals(expectedFls, velocities[2], EPSILON, "fls");
    assertEquals(expectedFla, velocities[3], EPSILON, "fla");
    assertEquals(expectedBls, velocities[4], EPSILON, "bls");
    assertEquals(expectedBla, velocities[5], EPSILON, "bla");
    assertEquals(expectedBrs, velocities[6], EPSILON, "brs");
    assertEquals(expectedBra, velocities[7], EPSILON, "bra");
  }

  @ParameterizedTest
  @CsvSource({
    "0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0",
    "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0",
    "1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0",
    "1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268, 0, 1, 0",
    "-1, 1.5707963268, -1, 1.5707963268, -1, 1.5707963268, -1, 1.5707963268, 0, -1, 0",
    "1, -1.5707963268, 1, -1.5707963268, 1, -1.5707963268, 1, -1.5707963268, 0, -1, 0",
    "1, 0.7853981634, 1, 0.7853981634, 1, 0.7853981634, 1, 0.7853981634, 0.7071067811847432, 0.7071067811847432, 0",
  })
  public void testToRobotVelocity(
      double frs,
      double fra,
      double fls,
      double fla,
      double bls,
      double bla,
      double brs,
      double bra,
      double expectedX,
      double expectedY,
      double expectedOmega) {
    double[] backToRobot =
        SwerveMath.calculateRobotVelocity(
            trackWidthOverFrameDiagonal,
            wheelBaseOverFrameDiagonal,
            frs,
            fra,
            fls,
            fla,
            bls,
            bla,
            brs,
            bra);
    assertEquals(expectedX, backToRobot[0], EPSILON, "x component is as expected");
    assertEquals(expectedY, backToRobot[1], EPSILON, "y component is as expected");
    assertEquals(expectedOmega, backToRobot[2], EPSILON, "omega component is as expected");
  }

  @ParameterizedTest
  @CsvSource({
    // "1.0, 2.0, 3.0", // deliberately fails to demonstrate test parameter validation
    // "0.0, 0.2, 3.0", // deliberately fails to demonstrate test parameter validation
    // "-0.8,  0.05, -0.5", // deliberately fails - desaturated speeds not reversible
    ".7, .7, 0",
    "0, 0, 0",
    "0.5, 0, 0",
    "1.0, 0, 0",
    "0, 0, 1",
    "0, 1.0, 0",
    "0.5, 0.5, 0",
    ".6, .6, 0",
    ".7, .7, 0",
    ".8, .2, 0",
    ".1, .1, 0",
    "0.25, 0.15, .25",
    "0.7071067812, 0.7071067812, 0",
    "0.0, 0.0, 0.2122065907891938",
    "0.0, 0.18867924528301888, 0.0",
    "0.09433962264150944, 0.09433962264150944, 0.0",
    "0.11320754716981132, 0.11320754716981132, 0.0",
    "0.1320754716981132, 0.1320754716981132, 0.0",
    "0.15094339622641512, 0.03773584905660378, 0.0",
    "0.01886792452830189, 0.01886792452830189, 0.0",
    "0.04716981132075472, 0.02830188679245283, 0.05305164769729845",
    "0.13341637381132077, 0.13341637381132077, 0.0",
    ".1, .1, .1",
    ".1, .1, .2",
    ".1, .1, .3",
    ".1, .1, .4",
    ".1, .1, .5",
    ".1, .1, .6",
    ".1, .1, .7",
    ".1, .1, .8",
    ".1, .2, .1",
    ".1, .2, .2",
    ".1, .2, .3",
    ".1, .2, .4",
    ".1, .2, .5",
    ".1, .2, .6",
    ".1, .2, .7",
    ".1, .2, .75",
  })
  public void testPowerToModuleAndBack(double xPwr, double yPwr, double omegaPwr) {
    // validate test parameters
    double speed = Math.hypot(xPwr, yPwr);
    assertTrue(
        speed <= (1 + EPSILON),
        "speed (magnitude of xy vector) must be between -1 and 1 - invalid test parameter - fix the test. (value: "
            + speed
            + ")");
    assertTrue(
        omegaPwr <= 1, "omega must be between -1 and 1 - invalid test parameter - fix the test.");
    assertTrue(
        omegaPwr >= -1, "omega must be between -1 and 1 - invalid test parameter - fix the test.");

    // @return an array of the calculated module velocities, in the format
    // [frs, fra, fls, fla, bls, bla, brs, bra] where
    // frs is the front right wheel speed from -1 to 1,
    // fra is the front right wheel angle, from -PI to PI, etc.
    double[] velocities =
        SwerveMath.calculateModuleVelocities(
            trackWidthOverFrameDiagonal, wheelBaseOverFrameDiagonal, xPwr, yPwr, omegaPwr);

    double frs = velocities[0];
    double fra = velocities[1];
    double fls = velocities[2];
    double fla = velocities[3];
    double bls = velocities[4];
    double bla = velocities[5];
    double brs = velocities[6];
    double bra = velocities[7];

    double[] backToRobot =
        SwerveMath.calculateRobotVelocity(
            trackWidthOverFrameDiagonal,
            wheelBaseOverFrameDiagonal,
            frs,
            fra,
            fls,
            fla,
            bls,
            bla,
            brs,
            bra);
    assertEquals(xPwr, backToRobot[0], EPSILON, "x power component");
    assertEquals(yPwr, backToRobot[1], EPSILON, "y power component");
    assertEquals(omegaPwr, backToRobot[2], EPSILON, "omega power component");
  }

  @ParameterizedTest
  @CsvSource({
    "0, 0, 0",
    "0, 0, 1",
    "0.5, 0, 0",
    "1.0, 0, 0",
    "2.0, 0, 0",
    "4, 0, 0",
    "5.2, 0, 0",
    "0, 1.0, 0",
    "0.5, 0.5, 0",
    ".6, .6, 0",
    ".7, .7, 0",
    ".8, .2, 0",
    ".1, .1, 0",
    "0.25, 0.15, .25",
    "0.7071067812, 0.7071067812, 0",
    "0, 0, 3.2812189937",
    "0, 0, 1",
    "0, 0, 2",
    "0, 0, 3",
    "0, 0, 4",
    "0, 0, 4.7",
    "1, 1, 1",
    "1, 1, 1.5",
    "1, 1, 2",
    "1, 1, 2.5",
    "1, 1, 3",
    "1, 2, 1",
    "1, 2, 1.5",
    "1, 2, 2",
    "1, 2, 2.5",
    "2, 1, 1",
    "2, 1, 1.5",
    "2, 1, 2",
    "2, 1, 2.5",
  })
  public void testMpsRadpsToModulesAndBack(double xMetres, double yMetres, double omegaRadPerSec) {
    double xPwr = xMetres / maxModuleSpeedMps;
    double yPwr = yMetres / maxModuleSpeedMps;
    double omegaPwr = omegaRadPerSec / maxOmegaRadPerSec;
    // just run the math test first...
    testPowerToModuleAndBack(xPwr, yPwr, omegaPwr);

    // now do the math but apply the conversions
    // @return an array of the calculated module velocities, in the format
    // [frs, fra, fls, fla, bls, bla, brs, bra] where
    // frs is the front right wheel speed from -1 to 1,
    // fra is the front right wheel angle, from -PI to PI, etc.
    var moduleVelocities =
        SwerveMath.calculateModuleVelocities(
            trackWidthOverFrameDiagonal, wheelBaseOverFrameDiagonal, xPwr, yPwr, omegaPwr);

    double frsPwr = moduleVelocities[0];
    double fraRad = moduleVelocities[1];
    double flsPwr = moduleVelocities[2];
    double flaRad = moduleVelocities[3];
    double blsPwr = moduleVelocities[4];
    double blaRad = moduleVelocities[5];
    double brsPwr = moduleVelocities[6];
    double braRad = moduleVelocities[7];

    //  @return array containing xPwr (-1, to 1), yPwr (-1 to 1), and w (rad/s)
    var robotVelocities =
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

    var xPwr_2 = robotVelocities[0];
    var convertedX = xPwr_2 * maxModuleSpeedMps;
    assertEquals(xMetres, convertedX, EPSILON, "x (m/s) (kinematics math only)");

    var yPwr_2 = robotVelocities[1];
    var convertedY = yPwr_2 * maxModuleSpeedMps;
    assertEquals(yMetres, convertedY, EPSILON, "y (m/s) (kinematics math only)");

    var omegaPwr_2 = robotVelocities[2];
    var convertedOmega = omegaPwr_2 * maxOmegaRadPerSec;
    assertEquals(omegaRadPerSec, convertedOmega, EPSILON, "omega (rad/s)");
  }

  @ParameterizedTest
  @CsvSource({
    "0, 0, 0",
    "0, 0, 1",
    "0.5, 0, 0",
    "1.0, 0, 0",
    "2.0, 0, 0",
    "4, 0, 0",
    "5.2, 0, 0",
    "0, 1.0, 0",
    "0.5, 0.5, 0",
    ".6, .6, 0",
    ".7, .7, 0",
    ".8, .2, 0",
    ".1, .1, 0",
    "0.25, 0.15, .25",
    "0.7071067812, 0.7071067812, 0",
    "0, 0, 3.2812189937",
    "0, 0, 1",
    "0, 0, 2",
    "0, 0, 3",
    "0, 0, 4",
    "0, 0, 4.7",
    "1, 1, 1",
    "1, 1, 1.5",
    "1, 1, 2",
    "1, 1, 2.5",
    "1, 1, 3",
    "1, 2, 1",
    "1, 2, 1.5",
    "1, 2, 2",
    "1, 2, 2.5",
    "2, 1, 1",
    "2, 1, 1.5",
    "2, 1, 2",
    "2, 1, 2.5",
  })
  public void testKinematics(double xMetres, double yMetres, double omegaRadPerSec) {

    // validate test parameters
    double speedMps = Math.hypot(xMetres, yMetres);
    assertTrue(
        speedMps <= maxModuleSpeedMps,
        "speed (magnitude of xy vector) must be between -"
            + maxModuleSpeedMps
            + " m/s and "
            + maxModuleSpeedMps
            + " m/s - invalid test parameter - fix the test.");
    assertTrue(
        omegaRadPerSec <= maxOmegaRadPerSec,
        "omega must be between -"
            + maxOmegaRadPerSec
            + " and "
            + maxOmegaRadPerSec
            + " rad/sec - invalid test parameter - fix the test.");
    assertTrue(
        omegaRadPerSec >= -maxOmegaRadPerSec,
        "omega must be between -"
            + maxOmegaRadPerSec
            + " and "
            + maxOmegaRadPerSec
            + " rad/sec - invalid test parameter - fix the test.");

    kin.calculateModuleVelocities(xMetres, yMetres, omegaRadPerSec);

    double frsMps = kin.getFrontRight().getSpeed();
    double fraDeg = kin.getFrontRight().getAngle();
    double flsMps = kin.getFrontLeft().getSpeed();
    double flaDeg = kin.getFrontLeft().getAngle();
    double brsMps = kin.getBackRight().getSpeed();
    double braDeg = kin.getBackRight().getAngle();
    double blsMps = kin.getBackLeft().getSpeed();
    double blaDeg = kin.getBackLeft().getAngle();

    // now generate them ourselves
    double[] discretized =
        SwerveMath.discretize(xMetres, yMetres, omegaRadPerSec, transScale, rotScale, normalScale);
    double[] pwr = {
      discretized[0] / maxModuleSpeedMps,
      discretized[1] / maxModuleSpeedMps,
      discretized[2] / maxOmegaRadPerSec
    };
    double[] modSpdsPwrRad =
        SwerveMath.calculateModuleVelocities(
            trackWidthOverFrameDiagonal, wheelBaseOverFrameDiagonal, pwr[0], pwr[1], pwr[2]);
    double[] modSpdsMpsDeg = {
      modSpdsPwrRad[0] * maxModuleSpeedMps,
      Math.toDegrees(modSpdsPwrRad[1]),
      modSpdsPwrRad[2] * maxModuleSpeedMps,
      Math.toDegrees(modSpdsPwrRad[3]),
      modSpdsPwrRad[4] * maxModuleSpeedMps,
      Math.toDegrees(modSpdsPwrRad[5]),
      modSpdsPwrRad[6] * maxModuleSpeedMps,
      Math.toDegrees(modSpdsPwrRad[7])
    };

    assertEquals(frsMps, modSpdsMpsDeg[0], "frs m/s");
    assertEquals(fraDeg, modSpdsMpsDeg[1], "frs deg");
    assertEquals(flsMps, modSpdsMpsDeg[2], "fls m/s");
    assertEquals(flaDeg, modSpdsMpsDeg[3], "fls deg");
    assertEquals(blsMps, modSpdsMpsDeg[4], "bls m/s");
    assertEquals(blaDeg, modSpdsMpsDeg[5], "bls deg");
    assertEquals(brsMps, modSpdsMpsDeg[6], "brs m/s");
    assertEquals(braDeg, modSpdsMpsDeg[7], "brs deg");
  }
}
