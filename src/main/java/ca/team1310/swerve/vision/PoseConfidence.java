package ca.team1310.swerve.vision;

/**
 * Enum to represent the confidence of a pose
 */
public enum PoseConfidence {
    /**
     * No confidence - do not use this pose data
     */
    NONE,
    /**
     * Low confidence - use this pose data with caution
     */
    LOW,
    /**
     * Medium confidence - use this pose data with some confidence
     */
    MEDIUM,
    /**
     * High confidence - use this pose data with high confidence
     */
    HIGH,
}
