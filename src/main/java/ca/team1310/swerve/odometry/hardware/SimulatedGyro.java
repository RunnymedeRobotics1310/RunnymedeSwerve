package ca.team1310.swervedrive.odometry.hardware;

import ca.team1310.swervedrive.SwerveTelemetry;
import ca.team1310.swervedrive.odometry.Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SimulatedGyro implements Gyro {

    private final Timer timer = new Timer();
    private double      lastTime;
    private double      roll  = 0;
    private double      pitch = 0;
    private double      yaw   = 0;

    public SimulatedGyro() {
        this.timer.start();
        this.lastTime = this.timer.get();
    }

    public double getYaw() {
        return this.yaw;
    }

    public double getPitch() {
        return this.pitch;
    }

    @Override
    public void zeroGyro() {
        this.yaw = 0;
    }

    public double getRoll() {
        return this.roll;
    }

    public void updateOdometryForSimulation(SwerveDriveKinematics kinematics, SwerveModuleState[] states, Pose2d[] modulePoses,
        Field2d field) {
        double change = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * (this.timer.get() - this.lastTime);
        this.yaw      += Units.radiansToDegrees(change);
        this.lastTime  = this.timer.get();
        field.getObject("XModules").setPoses(modulePoses);
    }

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry) {
        telemetry.gyroRawYawDegrees      = this.getYaw();
        telemetry.gyroAdjustedYawDegrees = this.getYaw();
        telemetry.gyroRawPitchDegrees    = this.getPitch();
        telemetry.gyroRawRollDegrees     = this.getRoll();
    }

}
