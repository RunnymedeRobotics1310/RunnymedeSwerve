package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;
import ca.team1310.swerve.gyro.hardware.SimulatedGyro;
import ca.team1310.swerve.utils.Coordinates;
import ca.team1310.swerve.vision.PoseEstimate;
import ca.team1310.swerve.vision.VisionPoseCallback;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;

/**
 * @author Tony Field
 * @since 2025-02-05 23:18
 */
public class FieldAwareSwerveDrive extends GyroAwareSwerveDrive {

    private final Field2d field;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveModulePosition[] modulePosition = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };
    private final SwerveModuleState[] moduleState = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
    };
    private final VisionPoseCallback visionPoseCallback;

    private final Notifier odometryUpdater = new Notifier(this::updateOdometry);
    private final Lock lock = new java.util.concurrent.locks.ReentrantLock();
    private static final double ODOMETRY_UPDATE_PERIOD_SECONDS = 0.02;

    public FieldAwareSwerveDrive(CoreSwerveConfig cfg, VisionPoseCallback callback) {
        super(cfg);
        this.visionPoseCallback = callback == null ? new VisionPoseCallback() {} : callback;
        this.field = new Field2d();

        // Set up pose estimator
        Rotation2d initialRotation = Rotation2d.fromDegrees(getYaw());
        SwerveModulePosition[] initialModulePositions = getSwerveModulePositions();
        System.out.println(
            "Initial module positions: " +
            Arrays.stream(initialModulePositions).map(SwerveModulePosition::toString).reduce("", (a, b) -> a + b)
        );
        Pose2d initialPose = new Pose2d(new Translation2d(0, 0), initialRotation);

        // Set up telemetry
        if (cfg.telemetry().enabled) {
            SmartDashboard.putData(this.field);
        }

        // set up the pose estimator
        var moduleLocations = new Translation2d[4];
        moduleLocations[0] = new Translation2d(
            cfg.frontLeftModuleConfig().location().getX(),
            cfg.frontLeftModuleConfig().location().getY()
        );
        moduleLocations[1] = new Translation2d(
            cfg.frontRightModuleConfig().location().getX(),
            cfg.frontRightModuleConfig().location().getY()
        );
        moduleLocations[2] = new Translation2d(
            cfg.backLeftModuleConfig().location().getX(),
            cfg.backLeftModuleConfig().location().getY()
        );
        moduleLocations[3] = new Translation2d(
            cfg.backRightModuleConfig().location().getX(),
            cfg.backRightModuleConfig().location().getY()
        );

        this.kinematics = new SwerveDriveKinematics(moduleLocations);

        this.estimator = new SwerveDrivePoseEstimator(
            this.kinematics,
            initialRotation,
            initialModulePositions,
            initialPose
        );

        this.odometryUpdater.startPeriodic(ODOMETRY_UPDATE_PERIOD_SECONDS);
        this.odometryUpdater.setName("RunnymedeSwerve Odometry");
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        var states = getModuleStates();
        for (int i = 0; i < states.length; i++) {
            modulePosition[i].distanceMeters = states[i].getPosition();
            modulePosition[i].angle = Rotation2d.fromDegrees(states[i].getAngle());
        }
        return modulePosition;
    }

    private void updateOdometry() {
        lock.lock();
        try {
            estimator.update(Rotation2d.fromDegrees(getYaw()), getSwerveModulePositions());
            if (isSimulation) {
                ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
                ((SimulatedGyro) gyro).updateOdometryForSimulation(measuredChassisSpeeds.omegaRadiansPerSecond);
            }
            PoseEstimate poseEstimate = visionPoseCallback.getPoseEstimate(getYaw(), getYawRate());
            if (poseEstimate != null) {
                if (poseEstimate.getStandardDeviations() == null) {
                    estimator.addVisionMeasurement(poseEstimate.getPose(), poseEstimate.getTimestampSeconds());
                } else {
                    estimator.addVisionMeasurement(
                        poseEstimate.getPose(),
                        poseEstimate.getTimestampSeconds(),
                        poseEstimate.getStandardDeviations()
                    );
                }
            }
        } finally {
            lock.unlock();
        }
    }

    public void resetOdometry(Pose2d pose) {
        lock.lock();
        try {
            estimator.resetPose(pose);
        } finally {
            lock.unlock();
        }
    }

    private Pose2d[] getModulePoses(Pose2d robotPose) {
        return Arrays.stream(getModuleStates())
            .map(m -> {
                Coordinates loc = m.getLocation();
                Translation2d trans = new Translation2d(loc.getX(), loc.getY());
                Rotation2d rot = Rotation2d.fromDegrees(m.getAngle());
                Transform2d tx = new Transform2d(trans, rot);
                return robotPose.plus(tx);
            })
            .toArray(Pose2d[]::new);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void updateTelemetry(SwerveTelemetry telemetry) {
        super.updateTelemetry(telemetry);

        Pose2d pose = estimator.getEstimatedPosition();
        field.setRobotPose(pose);
        ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
        telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
        telemetry.measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeeds.omegaRadiansPerSecond);
        telemetry.poseMetresX = pose.getTranslation().getX();
        telemetry.poseMetresY = pose.getTranslation().getY();
        telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();

        field.getObject("XModules").setPoses(getModulePoses(pose));
    }

    private SwerveModuleState[] getSwerveModuleStates() {
        var states = getModuleStates();
        for (int i = 0; i < states.length; i++) {
            moduleState[i].speedMetersPerSecond = states[i].getSpeed();
            moduleState[i].angle = Rotation2d.fromDegrees(states[i].getAngle());
        }
        return moduleState;
    }
}
