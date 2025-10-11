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

  private static double trackWidth = .58;
  private static double wheelBase = .67;

  private static double halfL = wheelBase / 2, halfW = trackWidth / 2;
  private static double[] moduleX = new double[] {+halfL, +halfL, -halfL, -halfL}; // FR, FL, BL, BR
  private static double[] moduleY = new double[] {-halfW, +halfW, +halfW, -halfW}; // FR, FL, BL, BR

  @Test
  public void test() {
    assertTrue(true, "true is true");
  }

  @ParameterizedTest
  @CsvSource({
    // "1.0, 2.0, 3.0", // deliberately fails to demonstrate test parameter validation
    // "0.0, 0.2, 3.0", // deliberately fails to demonstrate test parameter validation
    // "-0.8,  0.05, -0.5", // deliberately fails - desaturated speeds not reversible
    "0.5, 0, 0",
    "1.0, 0, 0",
    "0, 1.0, 0",
    "0.5, 0.5, 0",
    ".6, .6, 0",
    ".7, .7, 0",
    ".8, .2, 0",
    "0, 0, 0",
    ".1, .1, 0",
    "0.25, 0.15, .25",
  })
  public void testToModuleVelocitiesAndBack(double x, double y, double omega) {
    // validate test parameters
    double speed = Math.hypot(x, y);
    assertTrue(
        speed <= 1,
        "speed (magnitude of xy vector) must be between -1 and 1 - invalid test parameter - fix the test.");
    assertTrue(
        omega <= 1, "omega must be between -1 and 1 - invalid test parameter - fix the test.");
    assertTrue(
        omega >= -1, "omega must be between -1 and 1 - invalid test parameter - fix the test.");

    double[] velocities = SwerveMath.calculateModuleVelocities(trackWidth, wheelBase, x, y, omega);

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
            wheelBase, trackWidth, frs, fra, fls, fla, bls, bla, brs, bra);
    System.out.println("Expected: " + x + ", " + y + ", " + omega);
    System.out.println(
        "Actual: " + backToRobot[0] + ", " + backToRobot[1] + ", " + backToRobot[2] + "\n");
    assertEquals(omega, backToRobot[2], EPSILON, "omega power component");
    assertEquals(y, backToRobot[1], EPSILON, "y power component");
    assertEquals(x, backToRobot[0], EPSILON, "x power component");
  }

  @ParameterizedTest
  @CsvSource({
    "0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0",
    "0,0,0,0,0,0,0,0,0,0,0",
    "1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0",
    "0, 1, 0, 1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268, 1, 1.5707963268",
    "1, 1, 1, 1, 0.8151675871302798, 0.7417938611924496, 1.3765309341217336, 0.17529613185192702, 0.6147729285293413, 0.693159867086551, 0.1463848167086979",
    "0, 1, 0, 1, 1.5707963267948966, 1, 1.5707963267948966, 1, 1.5707963267948966, 1, 1.5707963267948966", // TODO: FIXME - TEST NOT PASSING DUE TO API BUG
  })
  public void testToModuleVelocities(
      double x,
      double y,
      double omega,
      double frs,
      double fra,
      double fls,
      double fla,
      double bls,
      double bla,
      double brs,
      double bra) {
    double[] velocities = SwerveMath.calculateModuleVelocities(trackWidth, wheelBase, x, y, omega);

    assertEquals(frs, velocities[0], EPSILON, "frs");
    assertEquals(fra, velocities[1], EPSILON, "fra");
    assertEquals(fls, velocities[2], EPSILON, "fls");
    assertEquals(fla, velocities[3], EPSILON, "fla");
    assertEquals(bls, velocities[4], EPSILON, "bls");
    assertEquals(bla, velocities[5], EPSILON, "bla");
    assertEquals(brs, velocities[6], EPSILON, "brs");
    assertEquals(bra, velocities[7], EPSILON, "bra");
  }

  @ParameterizedTest
  @CsvSource({"0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5,    0,   0", "0,0,0,0,0,0,0,0,0,0,0"})
  public void testToRobotVelocity(
      double frs,
      double fra,
      double fls,
      double fla,
      double bls,
      double bla,
      double brs,
      double bra,
      double x,
      double y,
      double omega) {
    //    double[] backToRobot =
    //        SwerveMath.calculateRobotVelocity(moduleX, moduleY, frs, fra, fls, fla, bls, bla, brs,
    // bra);
    //    assertEquals(x, backToRobot[0], EPSILON, "x component is as expected");
    //    assertEquals(y, backToRobot[1], EPSILON, "y component is as expected");
    //    assertEquals(omega, backToRobot[2], EPSILON, "omega component is as expected");
    double[] backToRobot =
        SwerveMath.calculateRobotVelocity(
            wheelBase, trackWidth, frs, fra, fls, fla, bls, bla, brs, bra);
    assertEquals(x, backToRobot[0], EPSILON, "x component is as expected");
    assertEquals(y, backToRobot[1], EPSILON, "y component is as expected");
    assertEquals(omega, backToRobot[2], EPSILON, "omega component is as expected");
  }
}
