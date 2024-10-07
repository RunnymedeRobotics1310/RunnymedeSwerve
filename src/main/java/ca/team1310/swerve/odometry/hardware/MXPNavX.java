package ca.team1310.swervedrive.odometry.hardware;

import ca.team1310.swervedrive.SwerveTelemetry;
import ca.team1310.swervedrive.odometry.Gyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class MXPNavX implements Gyro {
    private final AHRS navx;
    private double     rollOffset;
    private double     pitchOffset;
    private double     yawOffset;

    public MXPNavX() {
        this.navx        = new AHRS(SerialPort.Port.kMXP);
        this.rollOffset  = navx.getRoll();
        this.pitchOffset = navx.getPitch();
        this.yawOffset   = navx.getYaw();
    }

    @Override
    public void zeroGyro() {
        rollOffset  = navx.getRoll();
        pitchOffset = navx.getPitch();
        yawOffset   = navx.getYaw();
    }

    @Override
    public double getRoll() {
        return navx.getRoll() - rollOffset;
    }

    @Override
    public double getPitch() {
        return navx.getPitch() - pitchOffset;
    }

    @Override
    public double getYaw() {
        return navx.getYaw() - yawOffset;
    }

    @Override
    public void updateOdometryForSimulation(SwerveDriveKinematics kinematics, SwerveModuleState[] states, Pose2d[] modulePoses,
        Field2d field) {
    }

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry) {
        telemetry.gyroRawYawDegrees      = navx.getYaw();
        telemetry.gyroAdjustedYawDegrees = this.getYaw();
        telemetry.gyroRawPitchDegrees    = navx.getPitch();
        telemetry.gyroRawRollDegrees     = navx.getRoll();
    }
}
