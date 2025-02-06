/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.GyroAwareSwerveDrive;

/**
 * @author Tony Field
 * @since 2025-02-05 23:18
 */
public class FieldAwareSwerveDrive extends GyroAwareSwerveDrive {

    public FieldAwareSwerveDrive(CoreSwerveConfig cfg) {
        super(cfg);
    }

    public void updateTelemetry(SwerveTelemetry telemetry) {
        super.updateTelemetry(telemetry);
        // todo: these need to move to odometrySwerveDrive
        //            ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        //            telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
        //            telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
        //            telemetry.measuredChassisSpeeds[2] = measuredChassisSpeeds.omegaRadiansPerSecond;
    }
}
