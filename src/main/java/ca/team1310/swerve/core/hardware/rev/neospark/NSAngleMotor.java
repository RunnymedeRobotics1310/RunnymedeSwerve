/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.AngleMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * @author Tony Field
 * @since 2025-01-26 07:00
 */
public abstract class NSAngleMotor<T extends SparkBase> extends NSBase<T> implements AngleMotor {

    private static final double ANGLE_ENCODER_MAX_ERROR_DEGREES = 0.5;
    private static final int UPDATE_ENCODER_EVERY_N_CYCLES = 50 * 1; // approx every 1s
    private static final double MAX_ANGULAR_VELOCITY_FOR_ENCODER_UPDATE = 2; // degrees per second
    private double measuredPosition;
    private int cyclesSinceLastEncoderUpdate = 0;

    /**
     * Construct a properly configured angle motor.
     * @param spark The spark motor controller
     * @param cfg   The configuration of the motor
     */
    public NSAngleMotor(T spark, MotorConfig cfg, int robotPeriodMillis) {
        super(spark);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(cfg.inverted());
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.voltageCompensation(cfg.nominalVoltage());
        config.smartCurrentLimit(cfg.currentLimitAmps());
        config.closedLoopRampRate(cfg.rampRateSecondsZeroToFull());
        config.openLoopRampRate(cfg.rampRateSecondsZeroToFull());

        // Configure what the spark sends over CAN. Keep usage low.
        config.signals
            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .externalOrAltEncoderPositionAlwaysOn(false)
            .externalOrAltEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionAlwaysOn(false)
            .primaryEncoderVelocityAlwaysOn(false)
            .iAccumulationAlwaysOn(false);

        // angle motor signals
        config.signals
            .appliedOutputPeriodMs(robotPeriodMillis / 2) // todo: for debugging only????????
            .faultsPeriodMs(robotPeriodMillis) // report faults as they happen
            .primaryEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(robotPeriodMillis / 2);

        // Angle motor
        final double angleConversionFactor = 360 / cfg.gearRatio();
        // report in degrees not rotations
        config.encoder.positionConversionFactor(angleConversionFactor);
        // report in degrees per second not rotations per minute
        config.encoder.velocityConversionFactor(angleConversionFactor / 60);

        // configure PID controller
        config.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(cfg.p(), cfg.i(), cfg.d(), cfg.ff())
            .iZone(cfg.izone())
            .outputRange(-180, 180)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180);

        // send them to the motor
        doWithRetry(
            () ->
                spark.configure(
                    config,
                    SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters
                )
        );
    }

    public void periodic() {
        measuredPosition = (encoder.getPosition() + 360) % 360;
    }

    @Override
    public double getPosition() {
        return measuredPosition;
    }

    @Override
    public void setReferenceAngle(double degrees) {
        doWithRetry(() -> controller.setReference(degrees, SparkBase.ControlType.kPosition));
    }

    @Override
    public void setEncoderPosition(double actualAngleDegrees) {
        if (cyclesSinceLastEncoderUpdate++ < UPDATE_ENCODER_EVERY_N_CYCLES) {
            // only update the encoder position every so often to avoid spamming the CAN bus
            return;
        }

        double omega = Math.abs(encoder.getVelocity());
        if (omega > MAX_ANGULAR_VELOCITY_FOR_ENCODER_UPDATE) {
            // angle motor is moving too fast to update.
            return;
        }

        // update the encoder position
        cyclesSinceLastEncoderUpdate = 0;

        double error = Math.abs(measuredPosition - actualAngleDegrees);
        if (error < ANGLE_ENCODER_MAX_ERROR_DEGREES) {
            // no need to update the encoder position
            return;
        }

        String log = String.format(
            "Angle encoder %d position is off by more than %.2f degrees. Resetting to %.2f. Measured %.2f, error %.2f.",
            spark.getDeviceId(),
            ANGLE_ENCODER_MAX_ERROR_DEGREES,
            actualAngleDegrees,
            measuredPosition,
            error
        );
        System.out.println(log);
        doWithRetry(() -> encoder.setPosition(actualAngleDegrees));
    }
}
