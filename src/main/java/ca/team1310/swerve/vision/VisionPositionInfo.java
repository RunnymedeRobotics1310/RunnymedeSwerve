package ca.team1310.swerve.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A class that represents the location information from a vision system.
 */
public record VisionPositionInfo(
    Pose2d pose,
    double timestampSeconds,
    Matrix<N3, N1> deviation,
    PoseConfidence confidence,
    double odometryDistDelta
) {
    public String toString() {
        return String.format(
            "(%.2f,%.2f)m %.0fd %.0fms %s",
            pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getRotation().getDegrees(),
            timestampSeconds,
            confidence
        );
    }
}
