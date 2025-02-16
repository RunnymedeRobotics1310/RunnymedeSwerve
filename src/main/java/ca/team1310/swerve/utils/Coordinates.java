package ca.team1310.swerve.utils;

/**
 * Object representing the x and y coordinates of a point. Units are not specified.
 * @author Tony Field
 * @since 2025-02-05 18:21
 */
public class Coordinates {

    private final double x, y;

    /**
     * Create a new set of coordinates
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Coordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Get the x coordinate
     * @return x
     */
    public double getX() {
        return x;
    }

    /**
     * Get the y coordinate
     * @return y
     */
    public double getY() {
        return y;
    }
}
