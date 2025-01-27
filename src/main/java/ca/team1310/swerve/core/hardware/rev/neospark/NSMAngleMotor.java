/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

/**
 * @author Tony Field
 * @since 2025-01-26 07:07
 */
public class NSMAngleMotor extends NSAngleMotor<SparkMax> {
    public NSMAngleMotor(int canId, MotorConfig cfg,  int robotPeriodMillis) {
        super(new SparkMax(canId, SparkLowLevel.MotorType.kBrushless), cfg, robotPeriodMillis);
    }
}
