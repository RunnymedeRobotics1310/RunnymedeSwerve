package ca.team1310.swerve.core.config;

/**
 * Configuration for the encoder.
 * @param inverted true if the encoder is upside down with respect to its expected orientation
 * @param retrySeconds how long to wait before retrying communication with the encoder
 * @param retryCount how many times to retry communication with the encoder
 */
public record EncoderConfig(boolean inverted, double retrySeconds, int retryCount) {}
