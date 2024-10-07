package ca.team1310.swervedrive.core.hardware.cancoder;

import ca.team1310.swervedrive.SwerveTelemetry;
import ca.team1310.swervedrive.core.config.EncoderConfig;
import ca.team1310.swervedrive.core.AbsoluteAngleEncoder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;

public class CanCoder implements AbsoluteAngleEncoder {

    /**
     * The maximum amount of times the swerve encoder will attempt to configure itself if failures
     * occur.
     */
    private final int      maximumRetries;
    private final double   retryDelaySeconds;
    private final CANcoder encoder;
    private final double   absoluteEncoderOffset;

    public CanCoder(int canId, double absoluteEncoderOffsetDegrees, EncoderConfig encoderConfig) {
        this.encoder = new CANcoder(canId);
        this.encoder.clearStickyFaults();
        this.maximumRetries    = encoderConfig.retryCount();
        this.retryDelaySeconds = encoderConfig.retrySeconds();

        CANcoderConfigurator cfg = this.encoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration()); // resets to factory defaults
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(
                encoderConfig.inverted() ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive));

        // note, we aren't bothering to push this offset up into the encoder, we will just manage it
        // here in this class
        this.absoluteEncoderOffset = absoluteEncoderOffsetDegrees;
    }

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        telemetry.angleEncoderAbsoluteOffsetDegrees[moduleIndex]    = absoluteEncoderOffset;
        telemetry.moduleAbsoluteEncoderPositionDegrees[moduleIndex] = getPosition();
    }

    @Override
    public double getPosition() {
        boolean readingError = isNotHealthy();
        if (readingError) {
            return -1.0;
        }

        StatusSignal<Double> angle = encoder.getAbsolutePosition().refresh();

        int                  retryCount;
        for (retryCount = 0; retryCount < maximumRetries; retryCount++) {
            if (angle.getStatus() == StatusCode.OK) {
                break;
            }
            angle = angle.waitForUpdate(retryDelaySeconds);
        }
        if (angle.getStatus() != StatusCode.OK) {
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty after retrying "
                + maximumRetries + " times. " + angle.getStatus().getDescription(), false);
            return -1.0;
        }
        if (retryCount > 0) {
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " read successfully but required " + retryCount
                + " retries to get valid data.", false);
        }

        return (angle.getValue() * 360 - absoluteEncoderOffset + 360) % 360;
    }

    private boolean isNotHealthy() {
        MagnetHealthValue strength = encoder.getMagnetHealth().getValue();
        switch (strength) {
        case Magnet_Green:
            return false;
        case Magnet_Orange: {
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.", false);
            return false;
        }
        case Magnet_Invalid:
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " not usable - magnet INVALID.", false);
        case Magnet_Red:
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " not usable - magnet health RED.", false);
        }
        return true;
    }
}
