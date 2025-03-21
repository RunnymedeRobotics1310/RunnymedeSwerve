package ca.team1310.swerve.vision;

import static ca.team1310.swerve.core.config.TelemetryLevel.VERBOSE;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.FieldAwareSwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

public class LimelightAwareSwerveDrive extends FieldAwareSwerveDrive {

  private static final int OFFSET_POSE_X = 0;
  private static final int OFFSET_POSE_Y = 1;
  private static final int OFFSET_POSE_ROTATION_YAW = 5;
  private static final int OFFSET_TOTAL_LATENCY = 6;

  // Standard Deviations Used for most of the Pose Updates
  private static final Matrix<N3, N1> MEGATAG2_STDDEV = VecBuilder.fill(0.06, 0.06, 9999999);

  // Orientation publishers
  private final DoubleArrayPublisher llRobotOrientation;

  // MegaTags
  private final DoubleArraySubscriber llMegaTag;

  // Field Extent for tag validity
  private final double fieldExtentX;
  private final double fieldExtentY;

  // Data for telemetry publishing
  private Pose2d visPose = null;

  public LimelightAwareSwerveDrive(
      CoreSwerveConfig config, String limelightName, double fieldExtentX, double fieldExtentY) {
    super(config);

    this.fieldExtentX = fieldExtentX;
    this.fieldExtentY = fieldExtentY;

    final NetworkTable limelightNT =
        NetworkTableInstance.getDefault().getTable("limelight-" + limelightName);

    llRobotOrientation = limelightNT.getDoubleArrayTopic("robot_orientation_set").publish();
    llMegaTag = limelightNT.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  }

  @Override
  protected synchronized void updateOdometry() {
    super.updateOdometry();

    // Update the robot orientation
    llRobotOrientation.set(new double[] {getYaw(), 0, 0, 0, 0, 0});

    // Get the MegaTag data
    TimestampedDoubleArray megaTagAtomic = llMegaTag.getAtomic();
    double[] megaTagData = megaTagAtomic.value;
    long timestampMicros = megaTagAtomic.timestamp;

    // Check for valid data
    if (megaTagData != null && megaTagData.length >= 11) {

      // Extract the pose data & latency
      Pose2d botPose =
          new Pose2d(
              megaTagData[OFFSET_POSE_X],
              megaTagData[OFFSET_POSE_Y],
              Rotation2d.fromDegrees(megaTagData[OFFSET_POSE_ROTATION_YAW]));
      double totalLatencyMillis = megaTagData[OFFSET_TOTAL_LATENCY];

      // Ensure pose is on field
      if (botPose.getX() >= 0
          && botPose.getY() >= 0
          && botPose.getX() <= fieldExtentX
          && botPose.getY() <= fieldExtentY) {

        // Good data, let's update the pose estimator
        double latencySeconds = (timestampMicros / 1000000.0) - (totalLatencyMillis / 1000.0);
        getPoseEstimator().addVisionMeasurement(botPose, latencySeconds, MEGATAG2_STDDEV);
        visPose = botPose;
        return;
      }
    }

    // Fall through, no visPose
    visPose = null;
  }

  @Override
  public synchronized void updateTelemetry(SwerveTelemetry telemetry) {
    super.updateTelemetry(telemetry);

    telemetry.hasVisPose = visPose != null;

    if (telemetry.level == VERBOSE) {
      if (visPose != null) {
        telemetry.visionPoseX = visPose.getX();
        telemetry.visionPoseY = visPose.getY();
        telemetry.visionPoseHeading = visPose.getRotation().getDegrees();
      } else {
        telemetry.visionPoseX = 0;
        telemetry.visionPoseY = 0;
        telemetry.visionPoseHeading = 0;
      }
    }
  }
}
