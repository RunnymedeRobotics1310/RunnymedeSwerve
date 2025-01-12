package ca.team1310.swerve.core.config;

import ca.team1310.swerve.SwerveTelemetry;

/**
 * The Core swerve module configuration object.
 * @param wheelBaseMetres specify the distance between the front and back wheels
 * @param trackWidthMetres specify the distance between the left and right wheels
 * @param wheelRadiusMetres specify the radius of the wheels
 * @param robotPeriodSeconds specify the period of the robot. Usually 0.02s
 * @param maxAttainableModuleSpeedMetresPerSecond specify the maximum speed that the module can attain
 * @param maxAttainableTranslationSpeedMetresPerSecond specify the maximum speed that the robot can attain
 * @param maxAchievableRotationalVelocityRadiansPerSecond specify the maximum rotational velocity that the robot can attain
 * @param frontLeftModuleConfig specify the configuration of the front left module
 * @param frontRightModuleConfig specify the configuration of the front right module
 * @param backLeftModuleConfig specify the configuration of the back left module
 * @param backRightModuleConfig specify the configuration of the back right module
 * @param telemetry the swerve telemetry object
 */
public record CoreSwerveConfig(
    double wheelBaseMetres,
    double trackWidthMetres,
    double wheelRadiusMetres,
    double robotPeriodSeconds,
    double maxAttainableModuleSpeedMetresPerSecond,
    double maxAttainableTranslationSpeedMetresPerSecond,
    double maxAchievableRotationalVelocityRadiansPerSecond,
    ModuleConfig frontLeftModuleConfig,
    ModuleConfig frontRightModuleConfig,
    ModuleConfig backLeftModuleConfig,
    ModuleConfig backRightModuleConfig,
    SwerveTelemetry telemetry
) {}
