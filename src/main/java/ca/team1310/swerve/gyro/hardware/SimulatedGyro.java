package ca.team1310.swerve.gyro.hardware;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.gyro.Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * A simulated gyro that can be used in simulation mode.
 */
public class SimulatedGyro implements Gyro {

    private final Timer timer = new Timer();
    private double lastTime;
    private double roll = 0;
    private double pitch = 0;
    private double yaw = 0;
    private double yawRate = 0;

    /**
     * Constructs a new simulated gyro.
     */
    public SimulatedGyro() {
        this.timer.start();
        this.lastTime = this.timer.get();
    }

    public double getYawRate() {
        return this.yawRate;
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
    //    public void updateOdometryForSimulation(
    //        SwerveDriveKinematics kinematics,
    //        SwerveModuleState[] states,
    //        Pose2d[] modulePoses,
    //        Field2d field
    //    ) {
    //        double change = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * (this.timer.get() - this.lastTime);
    //        this.yaw += Units.radiansToDegrees(change);
    //        this.lastTime = this.timer.get();
    //        field.getObject("XModules").setPoses(modulePoses);
    //    }
    //
    //    @Override
    //    public void populateTelemetry(SwerveTelemetry telemetry) {
    //        if (telemetry.enabled) {
    //            telemetry.gyroRawYawDegrees = this.getYaw();
    //            telemetry.gyroAdjustedYawDegrees = this.getYaw();
    //            telemetry.gyroRawPitchDegrees = this.getPitch();
    //            telemetry.gyroRawRollDegrees = this.getRoll();
    //        }
    //    }
}
