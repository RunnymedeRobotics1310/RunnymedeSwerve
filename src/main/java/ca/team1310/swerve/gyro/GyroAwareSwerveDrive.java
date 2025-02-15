package ca.team1310.swerve.gyro;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.hardware.MXPNavX;
import ca.team1310.swerve.gyro.hardware.SimulatedGyro;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * @author Tony Field
 * @since 2025-01-31 17:02
 */
public class GyroAwareSwerveDrive extends CoreSwerveDrive {

    protected final Gyro gyro;

    /**
     * Create a new field-aware swerve drive
     *
     * @param cfg the core configuration for the swerve drive
     */
    public GyroAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro = RobotBase.isSimulation() ? new SimulatedGyro() : new MXPNavX();
        cfg.telemetry().gyro = gyro;
    }

    @Override
    public void zeroGyro() {
        gyro.zeroGyro();
    }

    @Override
    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public double getPitch() {
        return gyro.getPitch();
    }

    @Override
    public double getYaw() {
        return gyro.getYaw();
    }

    @Override
    public double getYawRate() {
        return gyro.getYawRate();
    }

    public void updateTelemetry(SwerveTelemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.gyroAdjustedYawDegrees = gyro.getYaw();
        telemetry.gyroRawPitchDegrees = gyro.getPitch();
        telemetry.gyroRawRollDegrees = gyro.getYaw();
    }
}
