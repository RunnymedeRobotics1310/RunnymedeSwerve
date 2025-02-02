package ca.team1310.swerve.core;

/**
 * Calculates swerve module setpoints to drive in a specified direction.
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

    private final ModuleState fr;
    private final ModuleState fl;
    private final ModuleState bl;
    private final ModuleState br;

    SwerveMath(double wheelBase, double trackWidth, double maxSpeedMps, double maxOmegaRadPerSec) {
        double hypot = Math.hypot(wheelBase, trackWidth);
        this.wheelbaseOverDiagonal = wheelBase / hypot;
        this.trackwidthOverDiagonal = trackWidth / hypot;
        this.maxSpeedMps = maxSpeedMps;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;
        this.fr = new ModuleState();
        this.fl = new ModuleState();
        this.bl = new ModuleState();
        this.br = new ModuleState();
    }

    /**
     * Calculate the module setpoints for the swerve drive when operating in robot-oriented mode
     * @param x desired forward velocity, in m/s (forward is positive)
     * @param y desired sideways velocity from in m/s (left is positive)
     * @param w desired angular velocity from in rad/s, (counter-clockwise is positive)
     */
    void calculateModuleSetpoints(double x, double y, double w) {
        calculateModuleSetpointsCWP(x, -y, -w);
    }

    /**
     * Calculate the module setpoints for the swerve drive when operating in robot-oriented mode
     * @param x desired forward velocity, in m/s (forward is positive)
     * @param y desired sideways velocity in m/s (right is positive)
     * @param w desired angular velocity in rad/s, (clockwise positive)
     */
    private void calculateModuleSetpointsCWP(double x, double y, double w) {
        // convert from mps to a scale of -1.0 to 1.0
        x /= maxSpeedMps;
        y /= maxSpeedMps;
        w /= maxOmegaRadPerSec;

        //
        // perform the main calculations
        //
        double a = x - w * wheelbaseOverDiagonal;
        double b = x + w * wheelbaseOverDiagonal;
        double c = y - w * trackwidthOverDiagonal;
        double d = y + w * trackwidthOverDiagonal;

        // calculate wheel speeds
        double frs = Math.hypot(b, c);
        double fls = Math.hypot(b, d);
        double bls = Math.hypot(a, d);
        double brs = Math.hypot(a, c);

        // calculate wheel angles
        double fra = (Math.atan2(b, c) * 180) / Math.PI;
        double fla = (Math.atan2(b, d) * 180) / Math.PI;
        double bla = (Math.atan2(a, d) * 180) / Math.PI;
        double bra = (Math.atan2(a, c) * 180) / Math.PI;

        //
        // update the final units
        //

        // normalize wheel speeds (cannot go faster than 1.0)
        double max = Math.max(frs, Math.max(fls, Math.max(bls, brs)));
        if (max > 1.0) {
            frs /= max;
            fls /= max;
            bls /= max;
            brs /= max;
        }

        // convert angle from -180 to -180 into -.5 to +.5
        fra /= 360;
        fla /= 360;
        bla /= 360;
        bra /= 360;

        // scale back to m/s and rad/s
        frs *= maxSpeedMps;
        fls *= maxSpeedMps;
        bls *= maxSpeedMps;
        brs *= maxSpeedMps;
        fra *= maxOmegaRadPerSec;
        fla *= maxOmegaRadPerSec;
        bla *= maxOmegaRadPerSec;
        bra *= maxOmegaRadPerSec;

        // set module setpoints
        fr.set(frs, fra);
        fl.set(fls, fla);
        bl.set(bls, bla);
        br.set(brs, bra);
    }

    ModuleState getFrontRight() {
        return fr;
    }

    ModuleState getFrontLeft() {
        return fl;
    }

    ModuleState getBackLeft() {
        return bl;
    }

    ModuleState getBackRight() {
        return br;
    }
}
