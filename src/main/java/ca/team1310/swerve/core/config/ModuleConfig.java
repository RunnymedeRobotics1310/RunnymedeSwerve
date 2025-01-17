package ca.team1310.swerve.core.config;

/**
 * Configuration for a single swerve module.
 * @param name the name of the module for internal reference and telemetry
 * @param xPositionMetres the x position of the module in metres with respect to the center of the robot
 * @param yPositionMetres the y position of the module in metres with respect to the center of the robot
 * @param wheelRadiusMetres the radius of the wheel in metres
 * @param driveMotorCanId the CAN ID of the drive motor
 * @param driveMotorConfig the configuration for the drive motor
 * @param angleMotorCanId the CAN ID of the angle motor
 * @param angleMotorConfig the configuration for the angle motor
 * @param angleEncoderCanId the CAN ID of the angle encoder
 * @param angleEncoderAbsoluteOffsetDegrees the offset of the absolute encoder in degrees
 * @param absoluteAngleEncoderConfig the configuration for the absolute angle encoder
 */
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
    EncoderConfig absoluteAngleEncoderConfig
) {}
