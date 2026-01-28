package ca.team1310.swerve.core.hardware.rev.neospark;

import static ca.team1310.swerve.core.CoreSwerveDrive.MANAGE_MODULES_PERIOD_MS;
import static ca.team1310.swerve.core.CoreSwerveDrive.TELEMETRY_UPDATE_PERIOD_MS;
import static ca.team1310.swerve.utils.SwerveUtils.clamp;

import ca.team1310.swerve.core.DriveMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * @author Tony Field
 * @since 2025-01-26 07:01
 */
public abstract class NSDriveMotor<T extends SparkBase> extends NSBase<T> implements DriveMotor {

  private double prevTargetMPS = 0;
  private final double maxMps;

  /**
   * Construct a properly configured drive motor.
   *
   * @param spark The spark motor controller
   * @param cfg The configuration of the motor
   * @param wheelRadiusMetres The radius of the wheel in metres
   * @param maxAttainableModuleSpeedMps The maximum attainable speed of the module's drive motor in
   *     metres per
   */
  public NSDriveMotor(
      T spark, MotorConfig cfg, double wheelRadiusMetres, double maxAttainableModuleSpeedMps) {
    super(spark);
    this.maxMps = maxAttainableModuleSpeedMps;
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(cfg.inverted());
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.voltageCompensation(cfg.nominalVoltage());
    config.smartCurrentLimit(cfg.currentLimitAmps());
    config.closedLoopRampRate(cfg.rampRateSecondsZeroToFull());
    config.openLoopRampRate(cfg.rampRateSecondsZeroToFull());

    // Configure what the spark sends over CAN. Keep usage low.
    // https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
    config
        .signals
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

    // drive motor signals
    config
        .signals
        .faultsPeriodMs(TELEMETRY_UPDATE_PERIOD_MS) // default is 250ms

        // applied output is used for diagnostics only
        .appliedOutputPeriodMs(TELEMETRY_UPDATE_PERIOD_MS) // default 10ms

        // position is used for odometry
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(MANAGE_MODULES_PERIOD_MS) // default is 20ms

        // velocity is used for telemetry and diagnostics
        .primaryEncoderVelocityAlwaysOn(true) // always on to prevent lag
        .primaryEncoderVelocityPeriodMs(TELEMETRY_UPDATE_PERIOD_MS); // default is 20ms

    // Drive motor
    double positionConversionFactor = (2 * Math.PI * wheelRadiusMetres) / cfg.gearRatio();
    // report in metres not rotations
    config.encoder.positionConversionFactor(positionConversionFactor);
    // report in metres per second not rotations per minute
    config.encoder.velocityConversionFactor(positionConversionFactor / 60);
    // reduce measurement lag --> read encoders every 2ms and use last 4 measurements for computing
    // velocity.  Default value is 100ms with a depth of 64
    config.encoder.quadratureMeasurementPeriod(2).quadratureAverageDepth(4);

    // configure PID controller - we want to speak in m/s
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(cfg.p(), cfg.i(), cfg.d(), cfg.ff())
        .iZone(cfg.izone())
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);

    // send them to the motor
    doWithRetry(
        () ->
            spark.configure(
                config,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public double getDistance() {
    return encoder.getPosition();
  }

  @Override
  public void setReferenceVelocity(double targetVelocityMPS) {
    final double targetVelocityMPSClamped = clamp(-maxMps, targetVelocityMPS, maxMps);
    if (Math.abs(targetVelocityMPSClamped - prevTargetMPS) < 1E-9) {
      return;
    }
    prevTargetMPS = targetVelocityMPSClamped;
    doWithRetry(
        () -> controller.setReference(targetVelocityMPSClamped, SparkBase.ControlType.kVelocity));
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getMeasuredVoltage() {
    return spark.getAppliedOutput();
  }
}
