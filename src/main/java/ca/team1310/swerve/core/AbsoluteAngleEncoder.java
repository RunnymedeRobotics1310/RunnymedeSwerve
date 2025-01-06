package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;

public interface AbsoluteAngleEncoder {

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

    void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex);
}
