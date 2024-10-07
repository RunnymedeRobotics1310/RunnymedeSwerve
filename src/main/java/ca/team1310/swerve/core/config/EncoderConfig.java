package ca.team1310.swervedrive.core.config;

public record EncoderConfig(
    boolean inverted,
    double retrySeconds,
    int retryCount) {
}
