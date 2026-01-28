package ca.team1310.swerve.core;

import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.core.hardware.cancoder.CanCoder;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFDriveMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMDriveMotor;
import ca.team1310.swerve.math.SwerveMath;
import ca.team1310.swerve.utils.Coordinates;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Notifier;

class SwerveModuleImpl implements SwerveModule {

  private static final double ANGLE_ENCODER_SYNC_PERIOD_MS = 500;
  private final Notifier encoderSynchronizer = new Notifier(this::syncAngleEncoder);

  private final String name;
  private final Coordinates location;
  private final DriveMotor driveMotor;
  private final AngleMotor angleMotor;
  private final AbsoluteAngleEncoder angleEncoder;
  private ModuleDirective desiredState = new ModuleDirective();
  private final ModuleState measuredState = new ModuleState();

  private final Alert driveMotorFaultPresent;
  private final Alert angleMotorFaultPresent;
  private final Alert angleEncoderFaultPresent;

  SwerveModuleImpl(ModuleConfig cfg, double maxAttainableModuleSpeedMps) {
    this.name = cfg.name();
    System.out.println(
        "Swerve ("
            + this.name
            + ") absolute angle encoder sync period: "
            + ANGLE_ENCODER_SYNC_PERIOD_MS
            + " ms");
    this.location = cfg.location();
    measuredState.setLocation(cfg.location());
    this.driveMotor = getDriveMotor(cfg, maxAttainableModuleSpeedMps);
    this.angleMotor = getAngleMotor(cfg);
    this.angleEncoder = getAbsoluteAngleEncoder(cfg);

    this.encoderSynchronizer.setName("RunnymedeSwerve Angle Encoder Sync " + name);
    this.encoderSynchronizer.startPeriodic(ANGLE_ENCODER_SYNC_PERIOD_MS / 1000);

    driveMotorFaultPresent =
        new Alert("Swerve Drive Motor [" + name + "] Fault Present", Alert.AlertType.kError);
    angleMotorFaultPresent =
        new Alert("Swerve Angle Motor [" + name + "] Fault Present", Alert.AlertType.kError);
    angleEncoderFaultPresent =
        new Alert("Swerve Angle Encoder [" + name + "] Fault Present!", Alert.AlertType.kError);
  }

  @Override
  public Coordinates getLocation() {
    return location;
  }

  private DriveMotor getDriveMotor(ModuleConfig cfg, double maxAttainableModuleSpeedMps) {
    return switch (cfg.driveMotorConfig().type()) {
      case NEO_SPARK_FLEX ->
          new NSFDriveMotor(
              cfg.driveMotorCanId(),
              cfg.driveMotorConfig(),
              cfg.wheelRadiusMetres(),
              maxAttainableModuleSpeedMps);
      case NEO_SPARK_MAX ->
          new NSMDriveMotor(
              cfg.driveMotorCanId(),
              cfg.driveMotorConfig(),
              cfg.wheelRadiusMetres(),
              maxAttainableModuleSpeedMps);
    };
  }

  private AngleMotor getAngleMotor(ModuleConfig cfg) {
    return switch (cfg.angleMotorConfig().type()) {
      case NEO_SPARK_FLEX -> new NSFAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig());
      case NEO_SPARK_MAX -> new NSMAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig());
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

  public synchronized void readState() {
    measuredState.setDesiredSpeed(desiredState.getSpeed());
    measuredState.setDesiredAngle(desiredState.getAngle());

    measuredState.setAngle(angleMotor.getPosition());
    measuredState.setPosition(driveMotor.getDistance());
  }

  public synchronized void readVerboseState() {
    measuredState.setVelocity(driveMotor.getVelocity());
    measuredState.setDriveOutputPower(driveMotor.getMeasuredVoltage());
    measuredState.setAbsoluteEncoderAngle(angleEncoder.getPosition());
  }

  public synchronized ModuleState getState() {
    return measuredState;
  }

  public synchronized void setDesiredState(ModuleDirective desiredState) {
    this.desiredState = desiredState;

    double currentHeadingDeg = angleMotor.getPosition();
    SwerveMath.optimizeWheelAngles(desiredState, currentHeadingDeg);
    SwerveMath.cosineCompensator(desiredState, currentHeadingDeg);

    // set drive motor speed
    driveMotor.setReferenceVelocity(desiredState.getSpeed());
    // update angle if drive speed > 0
    if (Math.abs(desiredState.getSpeed()) > 1E-9) {
      angleMotor.setReferenceAngle(desiredState.getAngle());
    }
  }

  /**
   * Check for active faults, manage alerts for them, and return status.
   *
   * @return true if there are active faults
   */
  public boolean checkFaults() {
    boolean driveFaults = driveMotor.hasFaults();
    boolean angleFaults = angleMotor.hasFaults();
    boolean angleEncoderFaults = angleEncoder.hasFaults();

    driveMotorFaultPresent.set(driveFaults);
    angleMotorFaultPresent.set(angleFaults);
    angleEncoderFaultPresent.set(angleEncoderFaults);

    return driveFaults | angleFaults | angleEncoderFaults;
  }
}
