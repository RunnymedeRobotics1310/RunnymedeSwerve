package ca.team1310.swerve.vision;

import static ca.team1310.swerve.core.config.TelemetryLevel.VERBOSE;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.config.GyroConfig;
import ca.team1310.swerve.odometry.FieldAwareSwerveDrive;
import ca.team1310.swerve.vision.config.LimelightConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

/**
 * A swerve drive that is aware of the Limelight vision system. This drive will use the Limelight to
 * update the robot's pose estimator with vision data.
 */
public class LimelightAwareSwerveDrive extends FieldAwareSwerveDrive {

  private static final int OFFSET_POSE_X = 0;
  private static final int OFFSET_POSE_Y = 1;
  private static final int OFFSET_POSE_ROTATION_YAW = 5;
  private static final int OFFSET_TOTAL_LATENCY = 6;
  private static final int OFFSET_TAG_COUNT = 7;
  private static final int OFFSET_AVG_TAG_DISTANCE = 9;

  // Orientation publishers
  private final DoubleArrayPublisher llRobotOrientation;

  // MegaTags
  private final DoubleArraySubscriber llMegaTag1;
  private final DoubleArraySubscriber llMegaTag2;

  // Field Extent for tag validity
  private final double fieldExtentX;
  private final double fieldExtentY;

  // Vision trust parameters
  private final double baseStdDevXY;
  private final double baseStdDevTheta;
  private final double distanceScaleFactor;
  private final double maxTrustDistanceMetres;
  private final double outlierRejectionThresholdMetres;
  private final int minTagCountForLowStdDev;

  // Data for telemetry publishing
  private Pose2d visPose = null;

  private boolean firstRun = true;

  /**
   * Create a new Limelight-aware swerve drive.
   *
   * @param config the core configuration for the swerve drive
   */
  public LimelightAwareSwerveDrive(
      CoreSwerveConfig config, GyroConfig gyroConfig, LimelightConfig limelightConfig) {

    super(config, gyroConfig);

    this.fieldExtentX = limelightConfig.fieldExtentX();
    this.fieldExtentY = limelightConfig.fieldExtentY();
    this.baseStdDevXY = limelightConfig.baseStdDevXY();
    this.baseStdDevTheta = limelightConfig.baseStdDevTheta();
    this.distanceScaleFactor = limelightConfig.distanceScaleFactor();
    this.maxTrustDistanceMetres = limelightConfig.maxTrustDistanceMetres();
    this.outlierRejectionThresholdMetres = limelightConfig.outlierRejectionThresholdMetres();
    this.minTagCountForLowStdDev = limelightConfig.minTagCountForLowStdDev();

    final NetworkTable limelightNT =
        NetworkTableInstance.getDefault().getTable("limelight-" + limelightConfig.limelightName());

    llRobotOrientation = limelightNT.getDoubleArrayTopic("robot_orientation_set").publish();
    llMegaTag1 = limelightNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    llMegaTag2 = limelightNT.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  }

  /**
   * Compute dynamic vision standard deviations based on tag count and average tag distance.
   *
   * @param tagCount number of tags visible
   * @param avgTagDistance average distance to visible tags in metres
   * @return standard deviation matrix, or null if the measurement should be rejected
   */
  private Matrix<N3, N1> computeVisionStdDevs(int tagCount, double avgTagDistance) {
    // Reject readings beyond max trust distance
    if (avgTagDistance > maxTrustDistanceMetres) {
      return null;
    }

    // Tag count penalty: single tags are less reliable
    double tagPenalty = tagCount < minTagCountForLowStdDev ? 2.0 : 1.0;

    // Formula: stddev = baseStdDev * (1 + distanceScaleFactor * distance^2) * tagPenalty
    double xyStdDev =
        baseStdDevXY * (1.0 + distanceScaleFactor * avgTagDistance * avgTagDistance) * tagPenalty;

    return VecBuilder.fill(xyStdDev, xyStdDev, baseStdDevTheta);
  }

  @Override
  protected final synchronized void updateOdometry() {
    super.updateOdometry();

    // Try to use MT1 pose yaw if it's present,
    if (firstRun) {
      double[] mt1Data = llMegaTag1.get();
      if (mt1Data != null && mt1Data.length >= 12) {
        double yaw = mt1Data[OFFSET_POSE_ROTATION_YAW];
        setYaw(yaw);
      }
      llMegaTag1.close();
      firstRun = false;
    }

    // Update the robot orientation
    llRobotOrientation.set(new double[] {getYaw(), 0, 0, 0, 0, 0});

    // Get the MegaTag data - can be multiple readings since we last checked
    Pose2d newVisPose = null;
    Pose2d currentPose = getPose();

    for (var megaTagAtomic : llMegaTag2.readQueue()) {
      double[] megaTagData = megaTagAtomic.value;
      long timestampMicros = megaTagAtomic.timestamp;

      // Check for valid data
      if (megaTagData != null && megaTagData.length >= 12) {

        // Extract the pose data & latency
        Pose2d botPose =
            new Pose2d(
                megaTagData[OFFSET_POSE_X],
                megaTagData[OFFSET_POSE_Y],
                Rotation2d.fromDegrees(megaTagData[OFFSET_POSE_ROTATION_YAW]));
        double totalLatencyMillis = megaTagData[OFFSET_TOTAL_LATENCY];
        int tagCount = (int) megaTagData[OFFSET_TAG_COUNT];
        double avgTagDistance = megaTagData[OFFSET_AVG_TAG_DISTANCE];

        // Ensure pose is on field
        if (botPose.getX() > 0
            && botPose.getY() > 0
            && botPose.getX() < fieldExtentX
            && botPose.getY() < fieldExtentY) {

          // Outlier rejection: skip poses too far from current estimate
          if (currentPose.getTranslation().getDistance(botPose.getTranslation())
              > outlierRejectionThresholdMetres) {
            continue;
          }

          // Compute dynamic standard deviations
          Matrix<N3, N1> stdDevs = computeVisionStdDevs(tagCount, avgTagDistance);
          if (stdDevs == null) {
            continue;
          }

          // Good data, let's update the pose estimator
          double latencySeconds = (timestampMicros / 1000000.0) - (totalLatencyMillis / 1000.0);
          getPoseEstimator().addVisionMeasurement(botPose, latencySeconds, stdDevs);
          newVisPose = botPose;
        }
      }
    }

    // Either no values were good, or we get the last one in the queue (most recent)
    visPose = newVisPose;
  }

  /**
   * Get the pose of the robot as determined by the vision system.
   *
   * @return the pose of the robot as determined by the vision system. Null if no vision pose is
   *     currently available.
   */
  public synchronized Pose2d getVisionPose() {
    return visPose;
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
