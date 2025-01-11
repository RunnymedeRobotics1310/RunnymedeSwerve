package ca.team1310.swerve.core.hardware.neosparkmax;

import ca.team1310.swerve.core.AngleMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


public class NSMAngleMotor extends NSMMotor implements AngleMotor {

    public NSMAngleMotor(int canId, MotorConfig cfg) {
        super(canId);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(cfg.inverted());
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.voltageCompensation(cfg.nominalVoltage());
        config.smartCurrentLimit(cfg.currentLimitAmps());
        config.closedLoopRampRate(cfg.rampRateSecondsZeroToFull());
        config.openLoopRampRate(cfg.rampRateSecondsZeroToFull());

        // default signals
        config.signals.absoluteEncoderPositionAlwaysOn(false)
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
        config.signals.primaryEncoderVelocityAlwaysOn(false)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20);

        // Angle motor
        final double angleConversionFactor = 360 / cfg.gearRatio();
        // report in degrees not rotations
        config.encoder.positionConversionFactor(angleConversionFactor);
        // report in degrees per second not rotations per minute
        config.encoder.velocityConversionFactor(angleConversionFactor / 60);
        // reduce measurement lag
        config.encoder
                .quadratureMeasurementPeriod(10)
                .quadratureAverageDepth(2);

        // configure PID controller
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(cfg.p(), cfg.i(), cfg.d(), cfg.ff())
                .iZone(cfg.izone())
                .outputRange(-180, 180)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);

        // send them to the motor
        doWithRetry(() -> motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters));
    }

    @Override
    public double getPosition() {
        return (encoder.getPosition() + 360) % 360;
    }

    @Override
    public void setReferenceAngle(double degrees) {
        doWithRetry(() -> controller.setReference(degrees, ControlType.kPosition));
    }

    @Override
    public void setEncoderPosition(double actualAngleDegrees) {
        if (encoder.getPosition() != actualAngleDegrees) {
            doWithRetry(() -> encoder.setPosition(actualAngleDegrees));
        }
    }
}
