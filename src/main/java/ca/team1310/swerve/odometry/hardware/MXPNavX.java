package ca.team1310.swerve.odometry.hardware;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.odometry.Gyro;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * A gyro that uses the NavX MXP to get the robot's orientation.
 */
public class MXPNavX implements Gyro {

    private final AHRS navx;
    private double rollOffset;
    private double pitchOffset;
    private double yawOffset;

    private double rollRaw;
    private double roll;
    private double pitchRaw;
    private double pitch;
    private double yawRaw;
    private double yaw;
    private double yawRate;
    private Rotation2d rotation;

    /**
     * Create a new MXPNavX gyro
     */
    public MXPNavX() {
        this.navx = new AHRS(NavXComType.kMXP_SPI);

        this.rollRaw = navx.getRoll();
        this.rollOffset = rollRaw;
        this.roll = rollRaw;

        this.pitchRaw = navx.getPitch();
        this.pitchOffset = pitchRaw;
        this.pitch = pitchRaw;

        this.yawRaw = navx.getYaw();
        this.yawOffset = yawRaw;
        this.yaw = yawRaw;

        this.yawRate = navx.getRate();
    }

    public void periodic() {
        readValues();
        computeValues();
    }

    /**
     * Read the values from the gyro
     */
    private void readValues() {
        this.rollRaw = navx.getRoll();
        this.pitchRaw = navx.getPitch();
        this.yawRaw = navx.getYaw();
        this.yawRate = navx.getRate();
    }

    /**
     * Compute derived values from the gyro
     */
    private void computeValues() {
        this.roll = rollRaw - rollOffset;
        this.pitch = pitchRaw - pitchOffset;
        this.yaw = yawRaw - yawOffset;
        this.rotation = Rotation2d.fromDegrees(yaw);
    }

    @Override
    public void zeroGyro() {
        rollOffset = rollRaw;
        pitchOffset = pitchRaw;
        yawOffset = yawRaw;
        computeValues();
    }

    @Override
    public double getRoll() {
        return roll;
    }

    @Override
    public double getPitch() {
        return pitch;
    }

    @Override
    public double getYaw() {
        return yaw;
    }

    @Override
    public double getYawRate() {
        return yawRate;
    }

    @Override
    public Rotation2d getRotation2d() {
        return rotation;
    }

    @Override
    public void updateOdometryForSimulation(
        SwerveDriveKinematics kinematics,
        SwerveModuleState[] states,
        Pose2d[] modulePoses,
        Field2d field
    ) {}

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry) {
        if (telemetry.enabled) {
            telemetry.gyroRawYawDegrees = yawRaw;
            telemetry.gyroAdjustedYawDegrees = yaw;
            telemetry.gyroRawPitchDegrees = pitchRaw;
            telemetry.gyroRawRollDegrees = rollRaw;
        }
    }
}
