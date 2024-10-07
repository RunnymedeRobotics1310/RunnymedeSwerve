package ca.team1310.swervedrive.vision;

import edu.wpi.first.math.geometry.Pose2d;

class PoseEstimate {
    public Pose2d        pose;
    public double        timestampSeconds;
    public double        latency;
    public int           tagCount;
    public double        tagSpan;
    public double        avgTagDist;
    public double        avgTagArea;
    public RawFiducial[] rawFiducials;

    public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
        int tagCount, double tagSpan, double avgTagDist,
        double avgTagArea, RawFiducial[] rawFiducials) {

        this.pose             = pose;
        this.timestampSeconds = timestampSeconds;
        this.latency          = latency;
        this.tagCount         = tagCount;
        this.tagSpan          = tagSpan;
        this.avgTagDist       = avgTagDist;
        this.avgTagArea       = avgTagArea;
        this.rawFiducials     = rawFiducials;
    }
}