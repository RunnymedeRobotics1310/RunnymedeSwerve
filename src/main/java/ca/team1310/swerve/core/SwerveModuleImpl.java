package ca.team1310.swerve.core;

import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.core.hardware.cancoder.CanCoder;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFDriveMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMDriveMotor;
import ca.team1310.swerve.utils.Coordinates;
import edu.wpi.first.wpilibj.Notifier;

class SwerveModuleImpl implements SwerveModule {

  private static final double ANGLE_ENCODER_SYNC_PERIOD_SECONDS = 0.5;

  private final String name;
  private final Coordinates location;
  private final DriveMotor driveMotor;
  private final AngleMotor angleMotor;
  private final AbsoluteAngleEncoder angleEncoder;
  private ModuleDirective desiredState = new ModuleDirective();
  private final ModuleState measuredState = new ModuleState();
  private final Notifier encoderSynchronizer = new Notifier(this::syncAngleEncoder);

  SwerveModuleImpl(ModuleConfig cfg, double maxAttainableModuleSpeedMps, int robotPeriodMillis) {
    this.name = cfg.name();
    this.location = cfg.location();
    measuredState.setLocation(cfg.location());
    this.driveMotor = getDriveMotor(cfg, maxAttainableModuleSpeedMps, robotPeriodMillis);
    this.angleMotor = getAngleMotor(cfg, robotPeriodMillis);
    this.angleEncoder = getAbsoluteAngleEncoder(cfg);
    encoderSynchronizer.setName("RunnymedeSwerve Angle Encoder Sync " + name);
    encoderSynchronizer.startPeriodic(ANGLE_ENCODER_SYNC_PERIOD_SECONDS);
  }

  @Override
  public Coordinates getLocation() {
    return location;
  }

  private DriveMotor getDriveMotor(
      ModuleConfig cfg, double maxAttainableModuleSpeedMps, int robotPeriodMillis) {
    return switch (cfg.driveMotorConfig().type()) {
      case NEO_SPARK_FLEX ->
          new NSFDriveMotor(
              cfg.driveMotorCanId(),
              cfg.driveMotorConfig(),
              cfg.wheelRadiusMetres(),
              maxAttainableModuleSpeedMps,
              robotPeriodMillis);
      default ->
          new NSMDriveMotor(
              cfg.driveMotorCanId(),
              cfg.driveMotorConfig(),
              cfg.wheelRadiusMetres(),
              maxAttainableModuleSpeedMps,
              robotPeriodMillis);
    };
  }

  private AngleMotor getAngleMotor(ModuleConfig cfg, int robotPeriodMillis) {
    return switch (cfg.angleMotorConfig().type()) {
      case NEO_SPARK_FLEX ->
          new NSFAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig(), robotPeriodMillis);
      default ->
          new NSMAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig(), robotPeriodMillis);
    };
  }

  private AbsoluteAngleEncoder getAbsoluteAngleEncoder(ModuleConfig cfg) {
    return new CanCoder(
        cfg.angleEncoderCanId(),
        cfg.angleEncoderAbsoluteOffsetDegrees(),
        cfg.absoluteAngleEncoderConfig());
  }

  private synchronized void syncAngleEncoder() {
    angleMotor.setEncoderPosition(angleEncoder.getPosition());
  }

  public String getName() {
    return name;
  }

  public synchronized void readState(boolean odometry, boolean telemetry) {
    measuredState.setDesiredSpeed(desiredState.getSpeed());
    measuredState.setDesiredAngle(desiredState.getAngle());

    if (odometry || telemetry) {
      measuredState.setAngle(angleMotor.getPosition());
      measuredState.setPosition(driveMotor.getDistance());
    }

    if (telemetry) {
      measuredState.setVelocity(driveMotor.getVelocity());
      measuredState.setDriveOutputPower(driveMotor.getMeasuredVoltage());
      measuredState.setAbsoluteEncoderAngle(angleEncoder.getPosition());
    }
  }

  public synchronized ModuleState getState() {
    return measuredState;
  }

  public synchronized void setDesiredState(ModuleDirective desiredState) {
    this.desiredState = desiredState;

    double currentHeadingDeg = angleMotor.getPosition();
    SwerveMath.optimizeWheelAngles(desiredState, currentHeadingDeg);
    SwerveMath.cosineCompensator(desiredState, currentHeadingDeg);

    driveMotor.setReferenceVelocity(desiredState.getSpeed());
    angleMotor.setReferenceAngle(desiredState.getAngle());
  }
}
