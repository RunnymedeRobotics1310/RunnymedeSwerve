package ca.team1310.swerve.core;

/**
 * Essential math utilities for swerve drive calculations.
 * <p>
 * Inspired by the work of FRC team 4048 - Redshift and their original implementation
 * <a href="https://github.com/FRC4048/Swerve-Drive-Library-Java/blob/master/src/main/java/org/usfirst/frc4048/swerve/math/SwerveMath.java">here</a>.
 * Simplified by Quentin Field (Team 1310 - Runnymede Robotics), Vanessa Field, and Tony Field in February 2025.
 * Calculates swerve module states to drive in a specified direction.
 * <p>
 * Static methods in this class are generally useful for all transformations. When performing module velocity
 * calculations repeatedly, create an instance of this class and use the instance method instead, to avoid
 * unnecessary calculations.
 * <p>
 * Module-specific calculations in this class assume a rectangular drive base, and that the center of rotation is
 * in the middle of the robot. An explanation of the principles behind the approach is included in this class, and
 * can be used as a basis for understanding how to adapt the calculations for other configurations.
 *
 * @author Tony Field
 * @author Quentin Field
 * @since 2025-02-01 19:20
 */
public class SwerveMath {

    private final double wheelBaseOverFrameDiagonal;
    private final double trackWidthOverFrameDiagonal;
    private final double maxSpeedMps;
    private final double maxOmegaRadPerSec;

    private final ModuleDirective fr;
    private final ModuleDirective fl;
    private final ModuleDirective bl;
    private final ModuleDirective br;

    /**
     * Construct an instance of SwerveMath for a rectangular robot. Preliminary calculations will be performed to
     * allow for optimal performance when calculating individual module velocities.
     *
     * @param wheelBase         the wheelbase of the drivetrain, (i.e. from the front to the back of the robot).
     *                          Units are not relevant.
     * @param trackWidth        the track width of the drivetrain (i.e. from the left to the right of the robot).
     *                          Units are not relevant.
     * @param maxSpeedMps       the maximum achievable speed of a module, in metres per second
     * @param maxOmegaRadPerSec the maximum achievable angular velocity of a module, in radians per second
     */
    SwerveMath(double wheelBase, double trackWidth, double maxSpeedMps, double maxOmegaRadPerSec) {
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
     * Calculate the module velocities for the swerve drive when operating in robot-oriented mode.  The individual
     * wheel vectors are stored inside this instance where they can be retrieved using the getter methods.  The
     * individual wheel vectors are returned in metres per second and degrees, counter-clockwise positive.
     *
     * @param x desired forward velocity, in m/s (forward is positive)
     * @param y desired sideways velocity from in m/s (left is positive)
     * @param w desired angular velocity from in rad/s, (counter-clockwise is positive)
     */
    void calculateAndStoreModuleVelocities(double x, double y, double w) {
        // convert from m/s to a scale of -1.0 to 1.0
        x /= maxSpeedMps;
        y /= maxSpeedMps;

        // convert from rad/s to a scale of -1.0 to 1.0
        w /= maxOmegaRadPerSec;

        var result = _calculateModuleVelocities(wheelBaseOverFrameDiagonal, trackWidthOverFrameDiagonal, x, y, w);

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
     * Calculate the module velocities for the swerve drive when operating in robot-oriented mode.
     * <p>
     * Units for x, y, and w are from -1.0 to 1.0, where 1.0 represents the maximum speed of the for each type of
     * movement.
     *
     * @param trackWidth the track width of the drivetrain (i.e. from the left to the right of the robot). Units
     *                   are not relevant.
     * @param wheelBase  the wheelbase of the drivetrain, (i.e. from the front to the back of the robot). Units
     *                   are not relevant.
     * @param x          desired forward velocity, from -1.0 to 1.0 where -1.0 is the minimum achievable value and
     *                   1.0 is the maximum achievable value. (forward is positive)
     * @param y          desired sideways velocity from -1.0 to 1.0 where -1.0 is the minimum achievable value and
     *                   1.0 is the maximum achievable value.  (left is positive)
     * @param w          desired angular velocity from -1.0 to 1.0 where -1.0 is the minimum achievable value and
     *                   1.0 is the maximum achievable value.  (counter-clockwise positive)
     * @return an array of the calculated module setpoints, in the format [frs, fra, fls, fla, bls, bla, brs, bra]
     * where frs is the front right wheel speed from -1 to 1, fra is the front right wheel angle, from -PI to PI, etc.
     */
    public static double[] calculateModuleVelocities(
        double trackWidth,
        double wheelBase,
        double x,
        double y,
        double w
    ) {
        double hypot = Math.hypot(wheelBase, trackWidth);
        double wheelBaseOverFrameDiagonal = wheelBase / hypot;
        double trackWidthOverFrameDiagonal = trackWidth / hypot;
        return _calculateModuleVelocities(wheelBaseOverFrameDiagonal, trackWidthOverFrameDiagonal, x, y, w);
    }

    /**
     * <p>
     * Calculate the module velocities for the swerve drive when operating in robot-oriented mode.
     * </p>
     * <p>
     * This is the core worker function for these calculations.
     * </p>
     *
     * <h1>Detailed Explanation of the Math</h1>
     *
     * <h2>Introduction</h2>
     * <p>Given desired robot velocity described as x (forward/backward motion with forward positive), y (left/right
     * motion with right positive), and w (i.e. omega) (rotation with clockwise positive), all with values in the
     * range of -1.0 (minimum possible) to 1.0 (maximum possible), compute the speed and angle of each module in
     * units of -1.0 to 1.0 for speed and -PI to PI for angle</p>
     * <p>Modules are labelled FR (front-right), FL, BL (back-left), and BR. The wheel base (distance between the
     * front and back wheels) and track width (distance between left and right wheels) is provided. The wheel
     * configuration is assumed to be a rectangle with wheels at the corners.</p>
     *
     * <h2>Approach</h2>
     * <p>We need to determine the wheel speed and angle for each wheel. This shall be the sum of two vectors: the
     * x,y translation vector + the tangential vector contributed by the rotation specified by w.</p>
     * <p>First, we compute the tangential vector for each wheel,</p>
     * <p>Next, we add the x component of the tangential vector to the input x velocity, and the y component of the
     * tangential vector to the input y velocity.</p>
     * <p>Once we have these vectors, we use basic trigonometry to compute the magnitude (i.e. module speed) and angle
     * (i.e. the individual module heading) for each wheel.</p>
     *
     * <h2>Setup</h2>
     * <p>Given, x (forward/backward motion, forward positive), y (left/right motion, left positive), and w (rotation,
     * counter-clockwise positive), we define a cartesian coordinate system with the origin at the center of the robot,
     * the x-axis pointing forward, and the y-axis pointing left.</p>
     * <p>We need to express our final values in polar coordinates, so we define a coordinate system with an angle of 0
     * pointing up (i.e. overlaid onto the X axis), counter-clockwise positive.</p>
     * <p>We define vectors from the origin to each wheel. The angle to the front-left module shall be named alpha,
     * and based on our coordinate system, it will be negative.</p>
     * <p>The tangential velocity vector associated with each wheel is defined as w * r, where r is the distance from
     * the origin to the module.</p>
     * <p>Because we have a rectangular base with known dimensions, r is half the diagonal of the drive base, and we
     * can get that using the pythagorean theorem.</p>
     *
     * <h2>Calculations</h2>
     * <p>We will calculate the wheel motion vectors by x and y components. As such, the x component of the front-left
     * wheel is defined as the input x value plus the x component of the tangential vector for the front-left wheel.
     * The y component of the front-left wheel is defined as the input y plus the y component of the tangential vector
     * for the front-left wheel. The same pattern applies for the other three wheels.</p>
     * <p>To get the x and y components for each wheel, we take the sine and cosine of their respective angle in the
     * coordinate system and multiply it by the value r.</p>
     * <p>We multiply these sin/cos values by w to get the x and y components of the tangential vector.</p>
     * <p>When we perform all of these calculations, we notice that the horizontal component for both back wheels is
     * the same, the horizontal component of the front wheels are the same, the vertical components of the right wheels
     * are the same, and the vertical components of the left wheels are the same. (This would not be true for a non-
     * rectangular drive base, or a drive base with a different number of modules. To expand this solution to a drive
     * base with different characteristics, this simplification can't be used, but the principles are the same).</p>
     * <p>Adding together the x and y values with their corresponding tangential components, we come up with the
     * following set of equations:</p>
     * <ul>
     *     <li>rear_horiz = y + w * cos(alpha - 180)</li>
     *     <li>front_horiz = y + w * cos(alpha)</li>
     *     <li>right_vert = x + w * sin(alpha)</li>
     *     <li>left_vert = x + w * sin(alpha - 180)</li>
     * </ul>
     * <p>Which simplifies to</p>
     * <ul>
     *     <li>rear_horiz = y - w * cos(alpha)</li>
     *     <li>front_horiz = y + w * cos(alpha)</li>
     *     <li>right_vert = x + w * sin(alpha)</li>
     *     <li>left_vert = x - w * sin(alpha)</li>
     * </ul>
     * <p>Back to our cartesian coordinates, we know that cos(alpha) is wheelbase / frame diagonal, and sin(alpha) is
     * trackwidth / frame diagonal. We simplify the above equations as follows:</p>
     * <ul>
     *     <li>rear_horiz = y - w * wheelbase / frame diagonal</li>
     *     <li>front_horiz = y + w * wheelbase / frame diagonal</li>
     *     <li>right_vert = x + w * trackwidth / frame diagonal</li>
     *     <li>left_vert = x - w * trackwidth / frame diagonal</li>
     * </ul>
     * <p>We now have the x and y components of all 4 of our wheel vectors! We are almost done.</p>
     *
     * <h2>Wheel Speeds</h2>
     * <p>The wheels speeds are simply the magnitude of the corresponding wheel vectors. These are:</p>
     * <ul>
     *     <li>frs: Math.hypot(front_horiz, right_vert)</li>
     *     <li>fls: Math.hypot(front_horiz, left_vert)</li>
     *     <li>bls: Math.hypot(rear_horiz, left_vert)</li>
     *     <li>brs: Math.hypot(rear_horiz, right_vert)</li>
     * </ul>
     * <p>We need to normalize the wheel speeds to ensure that none of them are greater than 1.0, because 1.0 is
     * the maximum achievable motor speed on the robot. Divide all values by the largest value if the largest
     * value is greater than 1.</p>
     * <p>That's it - wheel speed calculations done!</p>
     *
     * <h2>Wheel Angles</h2>
     * <p>To compute the angles for each wheel, we need the value in polar coordinates (ccw positive) of the angle,
     * which can be achieved using the arctan function as follows:</p>
     * <ul>
     *     <li>fra = Math.atan2(front_horiz, right_vert)</li>
     *     <li>fla = Math.atan2(front_horiz, left_vert)</li>
     *     <li>bla = Math.atan2(rear_horiz, left_vert)</li>
     *     <li>bra = Math.atan2(rear_horiz, right_vert)</li>
     * </ul>
     * <p>The output value is in radians. That's it, we are done!</p>
     *
     * <h2>Return Values</h2>
     * <p>To keep performance at a maximum, all 8 components of the module velocities are returned in a single
     * array in the format [frs, fra, fls, fla, bls, bla, brs, bra].</p>
     *
     * @param wheelBaseOverFrameDiagonal  the ratio of the wheelbase to the diagonal of the robot
     * @param trackWidthOverFrameDiagonal the ratio of the track width to the diagonal of the robot
     * @param x                           desired forward velocity, from -1.0 to 1.0 where -1.0 is the minimum
     *                                    achievable value and 1.0 is the maximum achievable value.
     *                                    (forward is positive)
     * @param y                           desired sideways velocity from -1.0 to 1.0 where -1.0 is the minimum
     *                                    achievable value and 1.0 is the maximum achievable value.  (left is positive)
     * @param w                           desired angular velocity from -1.0 to 1.0 where -1.0 is the minimum achievable
     *                                    value and 1.0 is the maximum achievable value.  (counter-clockwise positive)
     * @return an array of the calculated module velocities, in the format [frs, fra, fls, fla, bls, bla, brs, bra]
     * where frs is the front right wheel speed from -1 to 1, fra is the front right wheel angle, from -PI to PI, etc.
     */
    private static double[] _calculateModuleVelocities(
        double wheelBaseOverFrameDiagonal,
        double trackWidthOverFrameDiagonal,
        double x,
        double y,
        double w
    ) {
        // Compute the horiz and vert components of the wheel vectors
        double rear_horiz = y - w * wheelBaseOverFrameDiagonal;
        double front_horiz = y + w * wheelBaseOverFrameDiagonal;
        double right_vert = x + w * trackWidthOverFrameDiagonal;
        double left_vert = x - w * trackWidthOverFrameDiagonal;

        // calculate wheel speeds
        double frs = Math.hypot(front_horiz, right_vert);
        double fls = Math.hypot(front_horiz, left_vert);
        double bls = Math.hypot(rear_horiz, left_vert);
        double brs = Math.hypot(rear_horiz, right_vert);

        // normalize wheel speeds (cannot go faster than 1.0)
        double max = Math.max(frs, Math.max(fls, Math.max(bls, brs)));
        if (max > 1.0) {
            frs /= max;
            fls /= max;
            bls /= max;
            brs /= max;
        }

        // calculate wheel angles (in radians from -PI to PI)
        double fra = Math.atan2(front_horiz, right_vert);
        double fla = Math.atan2(front_horiz, left_vert);
        double bla = Math.atan2(rear_horiz, left_vert);
        double bra = Math.atan2(rear_horiz, right_vert);

        return new double[] { frs, fra, fls, fla, bls, bla, brs, bra };
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
     * Convert a vector from field-oriented to robot-oriented coordinates.
     *
     * @param fieldOrientedX      the x component of the vector (forward is positive, units don't matter)
     * @param fieldOrientedY      the y component of the vector (left is positive, units don't matter)
     * @param robotHeadingRadians the heading of the robot in radians (counter-clockwise is positive)
     * @return an array containing the x and y components of the vector in robot-oriented coordinates
     */
    public static double[] toRobotOriented(double fieldOrientedX, double fieldOrientedY, double robotHeadingRadians) {
        return rotate(fieldOrientedX, fieldOrientedY, -robotHeadingRadians);
    }

    /**
     * Convert a vector from robot-oriented to field-oriented coordinates.
     *
     * @param robotOrientedX      the x component of the vector (forward is positive, units don't matter)
     * @param robotOrientedY      the y component of the vector (left is positive, units don't matter)
     * @param robotHeadingRadians the heading of the robot in radians (counter-clockwise is positive)
     * @return an array containing the x and y components of the vector in field-oriented coordinates
     */
    public static double[] toFieldOriented(double robotOrientedX, double robotOrientedY, double robotHeadingRadians) {
        return rotate(robotOrientedX, robotOrientedY, robotHeadingRadians);
    }

    /**
     * Rotate the given vector by the specified angle.
     *
     * @param x            the x component of the vector. Units do not matter (but both x and y must be the same unit).
     * @param y            the y component of the vector. Units do not matter (but both x and y must be the same unit).
     * @param angleRadians the angle to rotate the vector by, in radians
     * @return a vector containing the rotated x and y values
     */
    public static double[] rotate(double x, double y, double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        double[] result = new double[2];
        result[0] = x * cos - y * sin;
        result[1] = x * sin + y * cos;
        return result;
    }
}
