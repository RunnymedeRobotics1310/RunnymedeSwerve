package ca.team1310.swervedrive.core.config;

public record MotorConfig(
    boolean inverted,
    int currentLimitAmps,
    double nominalVoltage,
    double rampRateSecondsZeroToFull,
    double gearRatio,
    double p,
    double i,
    double d,
    double ff,
    double izone) {
}
