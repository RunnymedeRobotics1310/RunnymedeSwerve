package ca.team1310.swerve.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

  private static double trackWidth = .45;
  private static double wheelBase = .65;

  @Test
  public void test() {
    assertTrue(true, "true is true");
  }

  @ParameterizedTest
  @CsvSource({"0.5,    0,   0", "1.0, 0, 0", "0, 1.0, 0", "0.75,    0.15,   0.25"
    // "1.0, 2.0, 3.0", // fails to demonstrate test parameter validation
    // "0.0, 0.2, 3.0", // fails to demonstrate test parameter validation
    //    "-0.8,  0.05, -0.5",  TODO: FIXME - TEST NOT PASSING DUE TO API BUG
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
            trackWidth, wheelBase, frs, fra, fls, fla, bls, bla, brs, bra);
    assertEquals(x, backToRobot[0], EPSILON, "x component");
    assertEquals(y, backToRobot[1], EPSILON, "y component");
    assertEquals(omega, backToRobot[2], EPSILON, "omega component");
  }

  @ParameterizedTest
  @CsvSource({
    "0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0",
    "0,0,0,0,0,0,0,0,0,0,0",
    "1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0"
    // "0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1",  TODO: FIXME - TEST NOT PASSING DUE TO API BUG
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
    double[] backToRobot =
        SwerveMath.calculateRobotVelocity(
            trackWidth, wheelBase, frs, fra, fls, fla, bls, bla, brs, bra);
    assertEquals(x, backToRobot[0], EPSILON, "x component is as expected");
    assertEquals(y, backToRobot[1], EPSILON, "y component is as expected");
    assertEquals(omega, backToRobot[2], EPSILON, "omega component is as expected");
  }
}
