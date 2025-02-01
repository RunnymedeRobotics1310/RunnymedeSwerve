/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.gyro;

import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.hardware.MXPNavX;
import ca.team1310.swerve.gyro.hardware.SimulatedGyro;
import edu.wpi.first.math.geometry.Pose2d;
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
    }

    public void periodic() {
        super.periodic();
        this.gyro.periodic();
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

    @Override
    public void resetOdometry(Pose2d pose) {
        throw new UnsupportedOperationException("Not supported in GyroAwareSwerveDrive");
    }

    @Override
    public Pose2d getPose() {
        throw new UnsupportedOperationException("Not supported in GyroAwareSwerveDrive");
    }
}
