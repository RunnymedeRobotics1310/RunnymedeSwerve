package ca.team1310.swerve.gyro;

import static ca.team1310.swerve.SwerveTelemetry.PREFIX;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.config.GyroConfig;
import ca.team1310.swerve.gyro.hardware.MXPNavX;
import ca.team1310.swerve.gyro.hardware.Pigeon2;
import ca.team1310.swerve.gyro.hardware.SimulatedGyro;
import ca.team1310.swerve.math.SwerveMath;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-01-31 17:02
 */
public class GyroAwareSwerveDrive extends CoreSwerveDrive {

  /** The gyro for the swerve drive */
  private final Gyro gyro;

  private final CoreSwerveConfig cfg;

  private boolean fieldOriented;
  private double fieldOrientedDesiredVx;
  private double fieldOrientedDesiredVy;

  /**
   * Create a new field-aware swerve drive
   *
   * @param cfg the core configuration for the swerve drive
   */
  public GyroAwareSwerveDrive(CoreSwerveConfig cfg, GyroConfig gyroConfig) {
    super(cfg);
    this.cfg = cfg;
    this.gyro = createGyro(gyroConfig);
    SmartDashboard.putData(PREFIX + "Gyro", this.gyro);
  }

  public final synchronized void driveRobotOriented(double x, double y, double w) {
    fieldOriented = false;
    super.driveRobotOriented(x, y, w);
  }

  public final synchronized void driveFieldOriented(double x, double y, double w) {
    fieldOriented = true;
    // if below minimum speed just stop rotating
    w = Math.abs(w) < MINIMUM_OMEGA_VALUE_RAD_PER_SEC ? 0 : w;

    // set desired speeds
    fieldOrientedDesiredVx = x;
    fieldOrientedDesiredVy = y;
    desiredOmega = w;
  }

  /*
   * Set the module states based on the desired speed and angle.
   */
  @Override
  protected final synchronized void updateModules() {
    if (fieldOriented) {
      double[] desired =
          SwerveMath.toRobotOriented(
              fieldOrientedDesiredVx, fieldOrientedDesiredVy, Math.toRadians(getYaw()));
      desiredVx = desired[0];
      desiredVy = desired[1];
    }
    super.updateModules();
  }

  /**
   * Create a gyro instance based on the configuration.
   *
   * @param gyroConfig the gyro configuration
   * @return the appropriate gyro instance
   */
  private Gyro createGyro(GyroConfig gyroConfig) {
    if (RobotBase.isSimulation()) {
      return new SimulatedGyro();
    }

    return switch (gyroConfig.type()) {
      case NAVX -> new MXPNavX();
      case PIGEON2 -> new Pigeon2(gyroConfig.canBusId(), gyroConfig.onlyYaw());
    };
  }

  @Override
  public synchronized void zeroGyro() {
    gyro.zeroGyro();
  }

  @Override
  public synchronized void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  @Override
  public synchronized double getRoll() {
    return gyro.getRoll();
  }

  @Override
  public synchronized double getPitch() {
    return gyro.getPitch();
  }

  @Override
  public synchronized double getYaw() {
    if (gyro == null) {
      System.out.println("Cannot get yaw, gyro is null");
      return 0;
    }
    return gyro.getYaw();
  }

  /**
   * Get the raw yaw value from hardware, without any zeroing/offset or normalization, in degrees.
   * Positive is CounterClockWise.
   *
   * @return the raw yaw of the robot, in degrees
   */
  public synchronized double getYawRaw() {
    if (gyro == null) {
      System.out.println("Cannot get yaw, gyro is null");
      return 0;
    }
    return gyro.getYawRaw();
  }

  @Override
  public synchronized double getYawRate() {
    return gyro.getYawRate();
  }

  @Override
  public synchronized void updateTelemetry(SwerveTelemetry telemetry) {
    // This is called during robot startup, and gyro may be null when this.super() finishes but
    // this class has not yet initialized the gyro, so perform null check.
    if (gyro != null) {
      gyro.isConnected();
    }

    super.updateTelemetry(telemetry);
  }

  @Override
  protected void updateGyroForSimulation() {
    // simulation
    if (isSimulation && gyro != null) {
      // note only capturing this twice in sim mode (which will run on a laptop, not a robot)
      var measuredRobotVelocity = getMeasuredRobotVelocity();
      ((SimulatedGyro) gyro).updateGyroForSimulation(measuredRobotVelocity[2]);
    }
  }

  double[] getRawDriveEncoders() {
    return getRawDriveMotorEncoders();
  }

  CoreSwerveConfig getCoreCfg() {
    return cfg;
  }
}
