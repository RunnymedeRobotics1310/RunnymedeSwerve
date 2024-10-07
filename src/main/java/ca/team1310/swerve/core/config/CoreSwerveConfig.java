package ca.team1310.swervedrive.core.config;

import ca.team1310.swervedrive.SwerveTelemetry;

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
    SwerveTelemetry telemetry) {
}
