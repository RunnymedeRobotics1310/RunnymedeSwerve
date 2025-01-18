package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;

/**
 * Interface for an absolute angle encoder. It exposes the methods that are necessary for an angle motor.
 */
public interface AbsoluteAngleEncoder {
    /**
     * Called at the start of each robot period. Used to read sensor values
     */
    void periodic();

    /**
     * Get the absolute position of the encoder.
     *
     * @return The absolute position of the encoder in degrees, from 0 to 360. Returns -1 on error.
     */
    double getPosition();

    /**
     * Get the velocity of the encoder.
     *
     * @return The velocity of the encoder in degrees per second. Returns -1 on error.
     */
    double getVelocity();

    /**
     * Populate the telemetry object with the encoder's data for this module.
     * @param telemetry The SwerveTelemetry object to populate.
     * @param moduleIndex The index of the module for which the telemetry should be populated.
     */
    void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex);
}
