/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

/**
 * @author Tony Field
 * @since 2025-01-26 07:17
 */
public class NSFAngleMotor extends NSAngleMotor<SparkFlex> {
    public NSFAngleMotor(int canId, MotorConfig cfg,  int robotPeriodMillis) {
        super(new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless), cfg, robotPeriodMillis);
    }
}
