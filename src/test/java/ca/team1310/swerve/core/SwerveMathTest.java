package ca.team1310.swerve.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * @author Tony Field
 * @since 2025-02-19 21:14
 */
public class SwerveMathTest {

    private static double trackWidth = .45;
    private static double wheelBase = .65;

    @Test
    public void test() {
        assertTrue(true, "true is true");
    }

    @Test
    public void testToModuleVelocitiesAndBack() {
        double x = 0.5;
        double y = 0;
        double omega = 0;

        double[] velocities = SwerveMath.calculateModuleVelocities(trackWidth, wheelBase, x, y, omega);

        double frs = velocities[0];
        double fra = velocities[1];
        double fls = velocities[2];
        double fla = velocities[3];
        double bls = velocities[4];
        double bla = velocities[5];
        double brs = velocities[6];
        double bra = velocities[7];

        assertEquals(0.5, frs, "frs");
        assertEquals(0.5, fls, "fls");
        assertEquals(0.5, brs, "brs");
        assertEquals(0.5, bls, "bls");

        assertEquals(0, fra, "fra");
        assertEquals(0, fla, "fla");
        assertEquals(0, bra, "bra");
        assertEquals(0, bla, "bla");

        double[] backToRobot = SwerveMath.calculateRobotVelocity(
            trackWidth,
            wheelBase,
            frs,
            fra,
            fls,
            fla,
            bls,
            bla,
            brs,
            bra
        );
        assertEquals(x, backToRobot[0], "x component is as expected");
        assertEquals(y, backToRobot[1], "y component is as expected");
        assertEquals(omega, backToRobot[2], "omega component is as expected");
    }
}
