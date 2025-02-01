package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;

/**
 * A field-aware swerve drive that uses a gyro and odometry to estimate the robot's pose on the field.
 */
public class FieldAwareSwerveDrive extends GyroAwareSwerveDrive {

    private static final double ODOMETRY_UPDATE_PERIOD_SECONDS = 0.02;

    private final Notifier notifier = new Notifier(this::updateOdometry);
    private final Lock lock = new java.util.concurrent.locks.ReentrantLock();
    private final Field2d field;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveTelemetry telemetry;

    /**
     * Create a new field-aware swerve drive
     * @param cfg the core configuration for the swerve drive
     */
    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.field = new Field2d();

        // Set up pose estimator
        Rotation2d initialRotation = Rotation2d.fromDegrees(gyro.getYaw());
        SwerveModulePosition[] initialModulePositions = getModulePositions();
        System.out.println(
            "Initial module positions: " +
            Arrays.stream(initialModulePositions).map(SwerveModulePosition::toString).reduce("", (a, b) -> a + b)
        );
        Pose2d initialPose = new Pose2d(new Translation2d(0, 0), initialRotation);

        // Set up telemetry
        if (cfg.telemetry().enabled) {
            SmartDashboard.putData(this.field);
        }
        this.telemetry = cfg.telemetry();

        this.estimator = new SwerveDrivePoseEstimator(kinematics, initialRotation, initialModulePositions, initialPose);
        this.notifier.startPeriodic(ODOMETRY_UPDATE_PERIOD_SECONDS);
    }

    public void periodic() {
        super.periodic();
        populateTelemetry();
    }

    /**
     * Add a vision measurement to the internal odometry system
     * @param pose the measured pose of the robot
     * @param timestampSeconds the timestamp of the measurement
     * @param deviation the deviation of the measurement
     */
    protected void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> deviation) {
        synchronized (lock) {
            estimator.addVisionMeasurement(pose, timestampSeconds, deviation);
        }
    }

    /**
     * Update the odometry tracking of the robot using module states and odometry measurements.
     */
    private void updateOdometry() {
        synchronized (lock) {
            estimator.update(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions());
            if (super.isSimulation) {
                gyro.updateOdometryForSimulation(
                    kinematics,
                    getModuleStates(),
                    getModulePoses(estimator.getEstimatedPosition()),
                    field
                );
            }
        }
    }

    public void resetOdometry(Pose2d pose) {
        synchronized (lock) {
            estimator.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), getModulePositions(), pose);
        }
    }

    public Pose2d getPose() {
        synchronized (lock) {
            return estimator.getEstimatedPosition();
        }
    }

    private void populateTelemetry() {
        if (telemetry.enabled) {
            Pose2d pose = estimator.getEstimatedPosition();
            field.setRobotPose(pose);
            gyro.populateTelemetry(telemetry);
            telemetry.poseMetresX = pose.getX();
            telemetry.poseMetresY = pose.getY();
            telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();
        }
    }
}
