package ca.team1310.swerve.core.hardware.neosparkmax;

import ca.team1310.swerve.core.DriveMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class NSMDriveMotor extends NSMMotor implements DriveMotor {

    public NSMDriveMotor(int canId, MotorConfig cfg, double wheelRadiusMetres) {
        super(canId);
        SparkMaxConfig config = new SparkMaxConfig();
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

        // drive motor signals
        config.signals
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20);

        // Drive motor
        final double positionConversionfactor = (2 * Math.PI * wheelRadiusMetres) / cfg.gearRatio();
        // report in metres not rotations
        config.encoder.positionConversionFactor(positionConversionfactor);
        // report in metres per second not rotations per minute
        config.encoder.velocityConversionFactor(positionConversionfactor / 60);
        // reduce measurement lag
        config.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);

        // configure PID controller
        config.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(cfg.p(), cfg.i(), cfg.d(), cfg.ff())
            .iZone(cfg.izone())
            .outputRange(-1, 1)
            .positionWrappingEnabled(false);

        // send them to the motor
        doWithRetry(
            () ->
                motor.configure(
                    config,
                    SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters
                )
        );
    }

    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public void setReferenceVelocity(double targetVelocityMPS) {
        doWithRetry(() -> controller.setReference(targetVelocityMPS, ControlType.kVelocity));
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
