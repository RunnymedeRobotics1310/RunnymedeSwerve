package ca.team1310.swerve.core;

/**
 * Calculates swerve module states to drive in a specified direction.
 *
 * The x axis is forward (positive) and backward (negative).
 * The y axis is left (positive) and right (negative).
 *
 * This class was inspired by the work of FRC team 4048 - Redshift. Their original implementation can be found here:
 * https://github.com/FRC4048/Swerve-Drive-Library-Java/blob/master/src/main/java/org/usfirst/frc4048/swerve/math/SwerveMath.java
 *
 * @author Tony Field
 * @since 2025-02-01 19:20
 */
class SwerveMath {

    private final double wheelbaseOverDiagonal;
    private final double trackwidthOverDiagonal;
    private final double maxSpeedMps;
    private final double maxOmegaRadPerSec;
    private final double maxOmegaDegPerSec;

    private final ModuleDirective fr;
    private final ModuleDirective fl;
    private final ModuleDirective bl;
    private final ModuleDirective br;

    SwerveMath(double wheelBase, double trackWidth, double maxSpeedMps, double maxOmegaRadPerSec) {
        double hypot = Math.hypot(wheelBase, trackWidth);
        this.wheelbaseOverDiagonal = wheelBase / hypot;
        this.trackwidthOverDiagonal = trackWidth / hypot;
        this.maxSpeedMps = maxSpeedMps;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;
        this.maxOmegaDegPerSec = Math.toDegrees(maxOmegaRadPerSec);
        this.fr = new ModuleDirective();
        this.fl = new ModuleDirective();
        this.bl = new ModuleDirective();
        this.br = new ModuleDirective();
    }

    /**
     * Calculate the module setpoints for the swerve drive when operating in robot-oriented mode
     * @param x desired forward velocity, in m/s (forward is positive)
     * @param y desired sideways velocity from in m/s (left is positive)
     * @param w desired angular velocity from in rad/s, (counter-clockwise is positive)
     */
    void calculateModuleSetpoints(double x, double y, double w) {
        // convert from m/s to a scale of -1.0 to 1.0
        x /= maxSpeedMps;
        y /= maxSpeedMps;

        // convert from rad/s to a scale of -1.0 to 1.0
        w /= maxOmegaRadPerSec;

        calculateModuleSetpointsCWP(x, -y, -w);
    }

    /**
     * Calculate the module setpoints for the swerve drive when operating in robot-oriented mode
     * @param x desired forward velocity, -1.0 to 1.0 (forward is positive)
     * @param y desired sideways velocity from -1.0 to 1.0 (right is positive)
     * @param w desired angular velocity from -1.0 to 1.0, (clockwise positive)
     */
    private void calculateModuleSetpointsCWP(double x, double y, double w) {
        //
        // perform the main calculations
        //
        double a = y - w * wheelbaseOverDiagonal;
        double b = y + w * wheelbaseOverDiagonal;
        double c = x - w * trackwidthOverDiagonal;
        double d = x + w * trackwidthOverDiagonal;

        // calculate wheel speeds (TODO - confirm: speeds are from -1.0 to 1.0)
        double frs = Math.hypot(b, c);
        double fls = Math.hypot(b, d);
        double bls = Math.hypot(a, d);
        double brs = Math.hypot(a, c);

        // calculate wheel angles (in radians from -PI to PI)
        double fra = Math.atan2(b, c);
        double fla = Math.atan2(b, d);
        double bla = Math.atan2(a, d);
        double bra = Math.atan2(a, c);

        //
        // normalize results
        //

        // normalize wheel speeds (cannot go faster than 1.0)
        double max = Math.max(frs, Math.max(fls, Math.max(bls, brs)));
        if (max > 1.0) {
            frs /= max;
            fls /= max;
            bls /= max;
            brs /= max;
        }

        // convert angle from -PI to PI into -.5 to +.5
        fra /= (2 * Math.PI); // 90 deg left
        fla /= (2 * Math.PI); // ok
        bla /= (2 * Math.PI); // 90 deg right
        bra /= (2 * Math.PI); // ok

        // hacks for now...
        fra /= 2;
        fla /= 2;
        bla /= 2;
        bra /= 2;

        //
        // convert to m/s and deg/s
        //
        frs *= maxSpeedMps;
        fls *= maxSpeedMps;
        bls *= maxSpeedMps;
        brs *= maxSpeedMps;
        fra *= maxOmegaDegPerSec;
        fla *= maxOmegaDegPerSec;
        bla *= maxOmegaDegPerSec;
        bra *= maxOmegaDegPerSec;

        // set module setpoints
        fr.set(frs, -fra);
        fl.set(fls, -fla);
        bl.set(bls, -bla);
        br.set(brs, -bra);
    }

    ModuleDirective getFrontRight() {
        return fr;
    }

    ModuleDirective getFrontLeft() {
        return fl;
    }

    ModuleDirective getBackLeft() {
        return bl;
    }

    ModuleDirective getBackRight() {
        return br;
    }
}
