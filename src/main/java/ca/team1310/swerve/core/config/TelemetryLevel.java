/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.core.config;

/**
 * @author Tony Field
 * @since 2025-02-16 10:29
 */
public enum TelemetryLevel {
    /**
     * The telemetry is disabled
     */
    NONE,
    /**
     * Only data passed into swerve is reported
     */
    INPUT,
    /**
     * Passed in data + calculated machine data is reported. Includes robot pose and module pose.
     */
    CALCULATED,
    /**
     * Same as calculated data and also displays data for human consumption
     */
    VERBOSE
}
