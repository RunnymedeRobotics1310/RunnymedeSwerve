package ca.team1310.swervedrive.core.config;

public record ModuleConfig(
    String name,
    double xPositionMetres,
    double yPositionMetres,
    double wheelRadiusMetres,
    int driveMotorCanId,
    MotorConfig driveMotorConfig,
    int angleMotorCanId,
    MotorConfig angleMotorConfig,
    int angleEncoderCanId,
    double angleEncoderAbsoluteOffsetDegrees,
    EncoderConfig absoluteAngleEncoderConfig,
    int angleMotorEncoderUpdateFrequency) {
}
