package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.hardware.MXPNavX;
import ca.team1310.swerve.odometry.hardware.SimulatedGyro;
import ca.team1310.swerve.utils.Cache;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A field-aware swerve drive that uses a gyro and odometry to estimate the robot's pose on the field.
 */
public class FieldAwareSwerveDrive extends CoreSwerveDrive {

    private final Gyro gyro;
    private final Cache<Rotation3d> gyroRotation3dCache;
    private final Field2d field;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveTelemetry telemetry;
    private final Notifier odometryThread;
    private final Lock odometryLock = new ReentrantLock();

    /**
     * Create a new field-aware swerve drive
     * @param cfg the core configuration for the swerve drive
     */
    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro = RobotBase.isSimulation() ? new SimulatedGyro() : new MXPNavX();
        long cacheTTL = (long) (cfg.robotPeriodSeconds() * 1000 * 0.45); // cache for 45% of the robot period
        this.gyroRotation3dCache = new Cache<>(this::_constructGyroRotation3d, cacheTTL);
        this.field = new Field2d();
        this.estimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
        );
        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.stop();
        // todo: move to its own config once tested
        final double odometryPeriodSeconds = cfg.robotPeriodSeconds();
        odometryThread.startPeriodic(odometryPeriodSeconds);

        SmartDashboard.putData(this.field);
        SmartDashboard.putData(this.gyro);
        this.telemetry = cfg.telemetry();
    }

    /**
     * Add a vision measurement to the internal odometry system
     * @param pose the measured pose of the robot
     * @param timestampSeconds the timestamp of the measurement
     * @param deviation the deviation of the measurement
     */
    protected void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> deviation) {
        try {
            odometryLock.lock();
            estimator.addVisionMeasurement(pose, timestampSeconds, deviation);
        } finally {
            odometryLock.unlock();
        }
    }

    /**
     * Update the odometry tracking of the robot using module states and odometry measurements.
     */
    protected void updateOdometry() {
        try {
            odometryLock.lock();
            gyroRotation3dCache.invalidate();
            estimator.update(gyro.getRotation2d(), getModulePositions());
            Pose2d robotPose = estimator.getEstimatedPosition();
            field.setRobotPose(robotPose);
            gyro.updateOdometryForSimulation(kinematics, getModuleStates(), getModulePoses(robotPose), field);
            populateTelemetry(robotPose);
        } finally {
            odometryLock.unlock();
        }
    }

    public void resetOdometry(Pose2d pose) {
        try {
            odometryLock.lock();
            estimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
            gyroRotation3dCache.invalidate();
        } finally {
            odometryLock.unlock();
        }
    }

    public Pose2d getPose() {
        try {
            odometryLock.lock();
            return estimator.getEstimatedPosition();
        } finally {
            odometryLock.unlock();
        }
    }

    public Rotation3d getGyroRotation3d() {
        // Avoid over-fetching from the gyro. Get value from cache.
        return gyroRotation3dCache.get();
    }

    /**
     * Compute the Rotation3d object from the gyro, bypassing the cache.
     * @return the Rotation3d object representing the gyro's orientation
     */
    private Rotation3d _constructGyroRotation3d() {
        return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
    }

    public void zeroGyro() {
        gyro.zeroGyro();
        gyroRotation3dCache.invalidate();
    }

    private void populateTelemetry(Pose2d pose) {
        gyro.populateTelemetry(telemetry);
        telemetry.poseMetresX = pose.getX();
        telemetry.poseMetresY = pose.getY();
        telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();
    }
}
