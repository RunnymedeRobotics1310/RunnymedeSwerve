package ca.team1310.swerve.core;

/**
 * @author Tony Field
 * @since 2025-02-01 19:19
 */
class ModuleState {

    private double speed = 0;
    private double angle = 0;

    /**
     * Set the angle and speed of the module
     * @param speed in m/s
     * @param angle in rad
     */
    void set(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    /**
     * Get the angle of the module
     * @return the angle of the module in radians
     */
    double getAngle() {
        return angle;
    }

    /**
     * Get the speed of the module
     * @return the speed of the module in m/s
     */
    double getSpeed() {
        return speed;
    }
}
