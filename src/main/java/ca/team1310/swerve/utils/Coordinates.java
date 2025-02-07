/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.utils;

/**
 * Object representign the x and y coordinates of a point.
 * @author Tony Field
 * @since 2025-02-05 18:21
 */
public class Coordinates {

    private final double x, y;

    public Coordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
