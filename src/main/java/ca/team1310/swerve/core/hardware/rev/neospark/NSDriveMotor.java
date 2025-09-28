package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.DriveMotor;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * @author Tony Field
 * @since 2025-01-26 07:01
 */
public abstract class NSDriveMotor<T extends SparkBase> extends NSBase<T> implements DriveMotor {

  private double prevTarget = 0;

  /**
   * Construct a properly configured drive motor.
   *
   * @param spark The spark motor controller
   * @param cfg The configuration of the motor
   * @param wheelRadiusMetres The radius of the wheel in metres
   * @param maxAttainableModuleSpeedMps The maximum attainable speed of the module's drive motor in
   *     metres per
   * @param robotPeriodMillis The period of the robot in milliseconds
   */
  public NSDriveMotor(
      T spark,
      MotorConfig cfg,
      double wheelRadiusMetres,
      double maxAttainableModuleSpeedMps,
      int robotPeriodMillis) {
    super(spark);
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(cfg.inverted());
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.voltageCompensation(cfg.nominalVoltage());
    config.smartCurrentLimit(cfg.currentLimitAmps());
    config.closedLoopRampRate(cfg.rampRateSecondsZeroToFull());
    config.openLoopRampRate(cfg.rampRateSecondsZeroToFull());

    // Configure what the spark sends over CAN. Keep usage low.
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
        .appliedOutputPeriodMs(robotPeriodMillis / 2) // todo: for debugging only????????
        .faultsPeriodMs(robotPeriodMillis) // default is 250ms
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(robotPeriodMillis / 2) // default is 20ms
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(robotPeriodMillis / 2); // default is 20ms

    // Drive motor
    final double positionConversionfactor = (2 * Math.PI * wheelRadiusMetres) / cfg.gearRatio();
    // report in metres not rotations
    config.encoder.positionConversionFactor(positionConversionfactor);
    // report in metres per second not rotations per minute
    config.encoder.velocityConversionFactor(positionConversionfactor / 60);
    // reduce measurement lag --> read encoders every 5ms and use last 4 measurements for computing
    // velocity
    config.encoder.quadratureMeasurementPeriod(5).quadratureAverageDepth(4);

    // configure PID controller - we want to speak in m/s
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(cfg.p(), cfg.i(), cfg.d(), cfg.ff())
        .iZone(cfg.izone())
        .outputRange(-maxAttainableModuleSpeedMps, maxAttainableModuleSpeedMps)
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
    if (Math.abs(targetVelocityMPS - prevTarget) < 1E-9) {
      return;
    }
    prevTarget = targetVelocityMPS;
    doWithRetry(() -> controller.setReference(targetVelocityMPS, SparkBase.ControlType.kVelocity));
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getMeasuredVoltage() {
    return spark.getAppliedOutput();
  }
}
