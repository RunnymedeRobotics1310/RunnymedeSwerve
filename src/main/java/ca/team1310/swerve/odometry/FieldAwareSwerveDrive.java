package ca.team1310.swerve.odometry;

import static ca.team1310.swerve.core.config.TelemetryLevel.*;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.ModuleState;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;
import ca.team1310.swerve.utils.Coordinates;
import ca.team1310.swerve.vision.PoseEstimate;
import ca.team1310.swerve.vision.VisionPoseCallback;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

/**
 * @author Tony Field
 * @since 2025-02-05 23:18
 */
public class FieldAwareSwerveDrive extends GyroAwareSwerveDrive {

  private final Field2d field;
  private final SwerveDrivePoseEstimator estimator;
  private final SwerveModulePosition[] modulePosition = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final VisionPoseCallback visionPoseCallback;
  private PoseEstimate visionPoseEstimate;

  private final Notifier odometryUpdater;
  private static final int UPDATE_ODOMETRY_EVERY_MILLIS = 20;

  private static final double VISION_HIGH_QUALITY_X = 0.1;
  private static final double VISION_HIGH_QUALITY_Y = 0.1;
  private static final double VISION_HIGH_QUALITY_HEADING = 1;

  /**
   * Create a new field-aware swerve drive. An optional vision pose callback can be provided to
   * allow the drive to use vision data for odometry.
   *
   * @param cfg the core configuration for the swerve drive
   * @param callback the vision pose callback
   */
  public FieldAwareSwerveDrive(CoreSwerveConfig cfg, VisionPoseCallback callback) {
    super(cfg);
    this.visionPoseCallback = callback == null ? new VisionPoseCallback() {} : callback;
    this.field = new Field2d();
    SmartDashboard.putData(field);

    // Set up pose estimator
    Rotation2d initialRotation = Rotation2d.fromDegrees(getYawRaw());
    SwerveModulePosition[] initialModulePositions = getSwerveModulePositions();
    System.out.println(
        "Initial module positions: "
            + Arrays.stream(initialModulePositions)
                .map(SwerveModulePosition::toString)
                .reduce("", (a, b) -> a + b));
    Pose2d initialPose = new Pose2d(new Translation2d(0, 0), initialRotation);

    // set up the pose estimator
    var moduleLocations = new Translation2d[4];
    moduleLocations[0] =
        new Translation2d(
            cfg.frontRightModuleConfig().location().getX(),
            cfg.frontRightModuleConfig().location().getY());
    moduleLocations[1] =
        new Translation2d(
            cfg.frontLeftModuleConfig().location().getX(),
            cfg.frontLeftModuleConfig().location().getY());
    moduleLocations[2] =
        new Translation2d(
            cfg.backLeftModuleConfig().location().getX(),
            cfg.backLeftModuleConfig().location().getY());
    moduleLocations[3] =
        new Translation2d(
            cfg.backRightModuleConfig().location().getX(),
            cfg.backRightModuleConfig().location().getY());

    this.estimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(moduleLocations),
            initialRotation,
            initialModulePositions,
            initialPose);

    this.odometryUpdater = new Notifier(this::updateOdometry);
    this.odometryUpdater.startPeriodic(UPDATE_ODOMETRY_EVERY_MILLIS / 1000.0);
    this.odometryUpdater.setName("RunnymedeSwerve Odometry");
  }

  private synchronized SwerveModulePosition[] getSwerveModulePositions() {
    var states = getModuleStates();
    for (int i = 0; i < states.length; i++) {
      modulePosition[i].distanceMeters = states[i].getPosition();
      modulePosition[i].angle = Rotation2d.fromDegrees(states[i].getAngle());
    }
    return modulePosition;
  }

  private synchronized void updateOdometry() {
    if (this.estimator == null) {
      System.out.println("Cannot update odometry - estimator is null");
      return;
    }

    // odometry
    estimator.update(Rotation2d.fromDegrees(getYawRaw()), getSwerveModulePositions());

    // vision
    visionPoseEstimate = visionPoseCallback.getPoseEstimate(getPose(), getYaw(), getYawRate());
    if (visionPoseEstimate != null) {
      if (visionPoseEstimate.getStandardDeviations() == null) {
        estimator.addVisionMeasurement(
            visionPoseEstimate.getPose(), visionPoseEstimate.getTimestampSeconds());
      } else {
        double[] stddevs = visionPoseEstimate.getStandardDeviations();

        if ((stddevs[0] <= VISION_HIGH_QUALITY_X
                && stddevs[1] <= VISION_HIGH_QUALITY_Y
                && stddevs[2] <= VISION_HIGH_QUALITY_HEADING)
            || DriverStation.isDisabled()) {
          resetOdometry(visionPoseEstimate.getPose());
        } else {
          Matrix<N3, N1> deviations = VecBuilder.fill(stddevs[0], stddevs[1], stddevs[2]);
          estimator.addVisionMeasurement(
              visionPoseEstimate.getPose(),
              visionPoseEstimate.getTimestampSeconds(),
              deviations);
        }
      }
    }
  }

  @Override
  public synchronized void zeroGyro() {
    super.zeroGyro();
    Pose2d oldPose = estimator.getEstimatedPosition();
    Pose2d newPose = new Pose2d(oldPose.getX(), oldPose.getY(), Rotation2d.fromDegrees(0));
    estimator.resetPose(newPose);
  }

  public synchronized void resetOdometry(Pose2d pose) {
    //setYaw(getPose().getRotation().getDegrees());
    estimator.resetPose(pose);
  }

  public synchronized Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public synchronized void updateTelemetry(SwerveTelemetry telemetry) {
    if (telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
      if (this.estimator == null) {
        System.out.println("Cannot update telemetry - estimator is null");
        return;
      }

      Pose2d pose = estimator.getEstimatedPosition();
      field.setRobotPose(pose);

      if (telemetry.level == VERBOSE) {
        telemetry.poseMetresX = pose.getTranslation().getX();
        telemetry.poseMetresY = pose.getTranslation().getY();
        telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();

        if (visionPoseEstimate != null) {
          telemetry.visionPoseX = visionPoseEstimate.getPose().getX();
          telemetry.visionPoseY = visionPoseEstimate.getPose().getY();
          telemetry.visionPoseHeading = visionPoseEstimate.getPose().getRotation().getDegrees();
        } else {
          telemetry.visionPoseX = 0;
          telemetry.visionPoseY = 0;
          telemetry.visionPoseHeading = 0;
        }
      }

      var states = getModuleStates();
      field.getObject("XModules").setPoses(asModulePoses(states, pose));
    }
    super.updateTelemetry(telemetry);
  }

  private static Pose2d[] asModulePoses(ModuleState[] states, Pose2d robotPose) {
    return Arrays.stream(states)
        .map(
            m -> {
              Coordinates loc = m.getLocation();
              Translation2d trans = new Translation2d(loc.getX(), loc.getY());
              Rotation2d rot = Rotation2d.fromDegrees(m.getAngle());
              Transform2d tx = new Transform2d(trans, rot);
              return robotPose.plus(tx);
            })
        .toArray(Pose2d[]::new);
  }
}
