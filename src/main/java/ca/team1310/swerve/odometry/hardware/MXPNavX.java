package ca.team1310.swerve.odometry.hardware;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.odometry.Gyro;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
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

    /**
     * Create a new MXPNavX gyro
     */
    public MXPNavX() {
        this.navx = new AHRS(NavXComType.kMXP_SPI); // or is it kMXP_UART?

        this.rollRaw = navx.getRoll();
        this.rollOffset = rollRaw;
        this.roll = rollRaw;

        this.pitchRaw = navx.getPitch();
        this.pitchOffset = pitchRaw;
        this.pitch = pitchRaw;

        this.yawRaw = navx.getYaw();
        this.yawOffset = yawRaw;
        this.yaw = yawRaw;
    }

    public void periodic() {
        this.rollRaw = navx.getRoll();
        this.pitchRaw = navx.getPitch();
        this.yawRaw = navx.getYaw();

        this.roll = rollRaw - rollOffset;
        this.pitch = pitchRaw - pitchOffset;
        this.yaw = yawRaw - yawOffset;
    }

    @Override
    public void zeroGyro() {
        rollOffset = rollRaw;
        pitchOffset = pitchRaw;
        yawOffset = yawRaw;
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
    public void updateOdometryForSimulation(
        SwerveDriveKinematics kinematics,
        SwerveModuleState[] states,
        Pose2d[] modulePoses,
        Field2d field
    ) {}

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry) {
        telemetry.gyroRawYawDegrees = yawRaw;
        telemetry.gyroAdjustedYawDegrees = yaw;
        telemetry.gyroRawPitchDegrees = pitchRaw;
        telemetry.gyroRawRollDegrees = rollRaw;
    }
}
