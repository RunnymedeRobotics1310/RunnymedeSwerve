package ca.team1310.swerve.core.hardware.neosparkflex;

import ca.team1310.swerve.core.AngleMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * A Neo motor controlled by a SparkFlex motor controller that is configured to control the angle of a swerve module.
 */
public class NSFAngleMotor extends NSFMotor implements AngleMotor {

    private final int canId;
    private double measuredPosition;

    /**
     * Create a new angle motor with the given CAN ID and configuration.
     *
     * @param canId the CAN ID of the motor
     * @param cfg the configuration of the motor
     */
    public NSFAngleMotor(int canId, MotorConfig cfg) {
        super(canId);
        this.canId = canId;
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(cfg.inverted());
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.voltageCompensation(cfg.nominalVoltage());
        config.smartCurrentLimit(cfg.currentLimitAmps());
        config.closedLoopRampRate(cfg.rampRateSecondsZeroToFull());
        config.openLoopRampRate(cfg.rampRateSecondsZeroToFull());

        // default signals
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
            .iAccumulationAlwaysOn(false)
            .appliedOutputPeriodMs(10)
            .faultsPeriodMs(20);

        // angle motor signals
        config.signals
            .primaryEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20);

        // Angle motor
        final double angleConversionFactor = 360 / cfg.gearRatio();
        // report in degrees not rotations
        config.encoder.positionConversionFactor(angleConversionFactor);
        // report in degrees per second not rotations per minute
        config.encoder.velocityConversionFactor(angleConversionFactor / 60);
        // reduce measurement lag
        config.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);

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
                sparkFlexMotorController.configure(
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
        doWithRetry(() -> controller.setReference(degrees, ControlType.kPosition));
    }

    @Override
    public void setEncoderPosition(double actualAngleDegrees) {
        final double maxDegreesOff = 0.5;
        if (Math.abs(measuredPosition - actualAngleDegrees) > maxDegreesOff) {
            String log = String.format(
                "Angle encoder %d position is off by more than %.2f degrees. Resetting encoder position to %.2f. Measured position is %.2f.",
                canId,
                maxDegreesOff,
                actualAngleDegrees,
                measuredPosition
            );
            System.out.println(log);
            doWithRetry(() -> encoder.setPosition(actualAngleDegrees));
        }
    }
}
