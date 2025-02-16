package ca.team1310.swerve.gyro;

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

    /**
     * The gyro for the swerve drive
     */
    protected final Gyro gyro;

    /**
     * Create a new field-aware swerve drive
     *
     * @param cfg the core configuration for the swerve drive
     */
    public GyroAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
        this.gyro = RobotBase.isSimulation() ? new SimulatedGyro() : new MXPNavX();
    }

    @Override
    public synchronized void zeroGyro() {
        gyro.zeroGyro();
    }

    @Override
    public synchronized double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public synchronized double getPitch() {
        return gyro.getPitch();
    }

    @Override
    public synchronized double getYaw() {
        return gyro.getYaw();
    }

    @Override
    public synchronized double getYawRate() {
        return gyro.getYawRate();
    }
}
