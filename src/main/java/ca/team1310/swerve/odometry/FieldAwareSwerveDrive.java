package ca.team1310.swerve.odometry;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.hardware.MXPNavX;
import ca.team1310.swerve.odometry.hardware.SimulatedGyro;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class FieldAwareSwerveDrive extends CoreSwerveDrive implements RunnymedeSwerveDrive {
    private final Gyro                     gyro;
    private final Field2d                  field;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveTelemetry          telemetry;
    private final Notifier                 odometryThread;
    private final Lock                     odometryLock = new ReentrantLock();

    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro      = RobotBase.isSimulation() ? new SimulatedGyro() : new MXPNavX();
        this.field     = new Field2d();
        this.estimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.stop();
        // todo: move to its own config once tested
        final double odometryPeriodSeconds = cfg.robotPeriodSeconds();
        odometryThread.startPeriodic(odometryPeriodSeconds);

        SmartDashboard.putData(this.field);
        SmartDashboard.putData(this.gyro);
        this.telemetry = cfg.telemetry();

    }

    protected void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> deviation) {
        try {
            odometryLock.lock();
            estimator.addVisionMeasurement(pose, timestampSeconds, deviation);
        }
        finally {
            odometryLock.unlock();
        }
    }

    protected void updateOdometry() {
        try {
            odometryLock.lock();
            estimator.update(gyro.getRotation2d(), getModulePositions());
            Pose2d robotPose = estimator.getEstimatedPosition();
            field.setRobotPose(robotPose);
            gyro.updateOdometryForSimulation(kinematics, getStates(), getModulePoses(robotPose), field);
            populateTelemetry(robotPose);
        }
        finally {
            odometryLock.unlock();
        }
    }

    public void resetOdometry(Pose2d pose) {
        try {
            odometryLock.lock();
            estimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
        }
        finally {
            odometryLock.unlock();
        }
    }

    public Pose2d getPose() {
        try {
            odometryLock.lock();
            return estimator.getEstimatedPosition();
        }
        finally {
            odometryLock.unlock();
        }
    }

    public Rotation3d getGyroRotation3d() {
        return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
    }

    public void zeroGyro() {
        gyro.zeroGyro();
    }

    private void populateTelemetry(Pose2d pose) {
        gyro.populateTelemetry(telemetry);
        telemetry.poseMetresX        = pose.getX();
        telemetry.poseMetresY        = pose.getY();
        telemetry.poseHeadingDegrees = pose.getRotation().getDegrees();
    }
}
