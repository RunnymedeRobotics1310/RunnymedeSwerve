package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.hardware.MXPNavX;
import ca.team1310.swerve.odometry.hardware.SimulatedGyro;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;

/**
 * A field-aware swerve drive that uses a gyro and odometry to estimate the robot's pose on the field.
 */
public class FieldAwareSwerveDrive extends CoreSwerveDrive {

    private final Gyro gyro;
    private final Field2d field;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveTelemetry telemetry;

    /**
     * Create a new field-aware swerve drive
     * @param cfg the core configuration for the swerve drive
     */
    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro = RobotBase.isSimulation() ? new SimulatedGyro() : new MXPNavX();
        this.field = new Field2d();

        // Set up pose estimator
        Rotation2d initialRotation = gyro.getRotation2d();
        if (initialRotation == null) {
            initialRotation = new Rotation2d();
            System.out.println("Gyro not connected, using default rotation");
        }
        SwerveModulePosition[] initialModulePositions = getModulePositions();
        System.out.println(
            "Initial module positions: " +
            Arrays.stream(initialModulePositions).map(SwerveModulePosition::toString).reduce("", (a, b) -> a + b)
        );
        Pose2d initialPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(gyro.getYaw()));
        System.out.println("Initial pose: " + initialPose);
        this.estimator = new SwerveDrivePoseEstimator(kinematics, initialRotation, initialModulePositions, initialPose);

        // Set up telemetry
        SmartDashboard.putData(this.field);
        SmartDashboard.putData(this.gyro);
        this.telemetry = cfg.telemetry();
    }

    public void periodic() {
        super.periodic();
        this.gyro.periodic();
        updateOdometry();
    }

    /**
     * Add a vision measurement to the internal odometry system
     * @param pose the measured pose of the robot
     * @param timestampSeconds the timestamp of the measurement
     * @param deviation the deviation of the measurement
     */
    protected void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> deviation) {
        estimator.addVisionMeasurement(pose, timestampSeconds, deviation);
    }

    /**
     * Update the odometry tracking of the robot using module states and odometry measurements.
     */
    protected void updateOdometry() {
        estimator.update(gyro.getRotation2d(), getModulePositions());
        Pose2d robotPose = estimator.getEstimatedPosition();
        field.setRobotPose(robotPose);
        gyro.updateOdometryForSimulation(kinematics, getModuleStates(), getModulePoses(robotPose), field);
        populateTelemetry(robotPose);
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void zeroGyro() {
        gyro.zeroGyro();
    }

    public double getGyroRoll() {
        return gyro.getRoll();
    }

    public double getGyroPitch() {
        return gyro.getPitch();
    }

    public double getGyroYaw() {
        return gyro.getYaw();
    }

    protected double getGyroYawRate() {
        return gyro.getYawRate();
    }

    private void populateTelemetry(Pose2d pose) {
        if (telemetry.enabled) {
            gyro.populateTelemetry(telemetry);
            telemetry.poseMetresX = pose.getX();
            telemetry.poseMetresY = pose.getY();
            telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();
        }
    }
}
