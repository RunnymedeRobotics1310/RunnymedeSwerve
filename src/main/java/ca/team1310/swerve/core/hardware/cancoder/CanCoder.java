package ca.team1310.swerve.core.hardware.cancoder;

import static edu.wpi.first.units.Units.*;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.AbsoluteAngleEncoder;
import ca.team1310.swerve.core.config.EncoderConfig;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;

/**
 * CanCoder implementation of AbsoluteAngleEncoder. Uses WPILib's Alert system to report errors and warnings.
 */
public class CanCoder implements AbsoluteAngleEncoder {

    /**
     * The maximum amount of times the swerve encoder will attempt to configure itself if failures
     * occur.
     */
    private final int maximumRetries;
    private final double retryDelaySeconds;
    private final double absoluteEncoderOffset;

    private final StatusSignal<MagnetHealthValue> magnetHealth;
    private final StatusSignal<Angle> angle;
    private final StatusSignal<AngularVelocity> velocity;

    private final Alert magnetFieldLessThanIdeal;
    private final Alert readingFaulty;
    private final Alert readingIgnored;
    private final Alert cannotConfigureEncoder;
    private double measuredPosition;
    private double measuredVelocity;

    /**
     * Construct a new CanCoder with the specified configuration and offsets.
     * @param canId the canbus id of the CanCoder
     * @param absoluteEncoderOffsetDegrees the measured offset of the encoder in degrees
     * @param encoderConfig the configuration of the encoder
     */
    public CanCoder(int canId, double absoluteEncoderOffsetDegrees, EncoderConfig encoderConfig) {
        CANcoder encoder = new CANcoder(canId);
        encoder.clearStickyFaults();

        // configure
        this.maximumRetries = encoderConfig.retryCount();
        this.retryDelaySeconds = encoderConfig.retrySeconds();
        this.absoluteEncoderOffset = absoluteEncoderOffsetDegrees;

        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)).withSensorDirection(
            encoderConfig.inverted()
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive
        );

        StatusCode error = encoder.getConfigurator().apply(configuration);

        cannotConfigureEncoder = new Alert(
            "Encoders",
            "Failure to apply configuration to CANCoder " + encoder.getDeviceID(),
            Alert.AlertType.kWarning
        );
        cannotConfigureEncoder.set(error != StatusCode.OK);

        // measurements
        this.angle = encoder.getAbsolutePosition();
        this.velocity = encoder.getVelocity();

        // magnet health
        magnetHealth = encoder.getMagnetHealth();
        magnetFieldLessThanIdeal = new Alert(
            "Encoders",
            "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.",
            Alert.AlertType.kWarning
        );
        readingFaulty = new Alert(
            "Encoders",
            "CANCoder " + encoder.getDeviceID() + " reading was faulty.",
            Alert.AlertType.kWarning
        );
        readingIgnored = new Alert(
            "Encoders",
            "CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.",
            Alert.AlertType.kWarning
        );
    }

    public void periodic() {
        this.measuredPosition = calculatePosition();
        this.measuredVelocity = velocity.refresh().getValue().in(DegreesPerSecond);
    }

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        telemetry.angleEncoderAbsoluteOffsetDegrees[moduleIndex] = absoluteEncoderOffset;
        telemetry.moduleAbsoluteEncoderPositionDegrees[moduleIndex] = measuredPosition;
    }

    @Override
    public double getPosition() {
        return measuredPosition;
    }

    private double calculatePosition() {
        MagnetHealthValue strength = magnetHealth.refresh().getValue();

        magnetFieldLessThanIdeal.set(strength != MagnetHealthValue.Magnet_Green);
        if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red) {
            readingFaulty.set(true);
            return -1;
        } else {
            readingFaulty.set(false);
        }

        angle.refresh();

        for (int retryCount = 0; retryCount < maximumRetries; retryCount++) {
            if (angle.getStatus() == StatusCode.OK) {
                break;
            }
            angle.waitForUpdate(retryDelaySeconds);
        }

        if (angle.getStatus() != StatusCode.OK) {
            readingIgnored.set(true);
        } else {
            readingIgnored.set(false);
        }

        return angle.getValue().in(Degrees) - absoluteEncoderOffset;
    }

    @Override
    public double getVelocity() {
        return measuredVelocity;
    }
}
