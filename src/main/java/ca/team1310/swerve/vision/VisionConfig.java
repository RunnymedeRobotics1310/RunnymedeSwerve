package ca.team1310.swerve.vision;

public record VisionConfig(
    long pipelineAprilTagDetect,
    long camModeVision,
    double fieldExtentMetresX,
    double fieldExtentMetresY,
    double maxAmbiguity,
    double highQualityAmbiguity,
    double maxVisposeDeltaDistanceMetres) {
}
