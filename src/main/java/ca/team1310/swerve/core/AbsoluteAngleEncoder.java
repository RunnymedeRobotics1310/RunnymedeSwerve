package ca.team1310.swerve.core;

/**
 * Interface for an absolute angle encoder. It exposes the methods that are necessary for an angle motor.
 */
public interface AbsoluteAngleEncoder {
    /**
     * Get the absolute location of the encoder.
     *
     * @return The absolute location of the encoder in degrees, from 0 to 360. Returns -1 on error.
     */
    double getPosition();
}
