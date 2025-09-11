package ca.team1310.swerve.gyro;

import static ca.team1310.swerve.SwerveTelemetry.PREFIX;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.CoreSwerveDrive;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.core.config.GyroConfig;
import ca.team1310.swerve.gyro.hardware.MXPNavX;
import ca.team1310.swerve.gyro.hardware.Pigeon2;
import ca.team1310.swerve.gyro.hardware.SimulatedGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-01-31 17:02
 */
public class GyroAwareSwerveDrive extends CoreSwerveDrive {

  /** The gyro for the swerve drive */
  private final Gyro gyro;

  /**
   * Create a new field-aware swerve drive
   *
   * @param cfg the core configuration for the swerve drive
   */
  public GyroAwareSwerveDrive(CoreSwerveConfig cfg) {
    super(cfg);
    this.gyro = createGyro(cfg.gyroConfig());
    SmartDashboard.putData(PREFIX + "Gyro", this.gyro);
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
   * Get the raw yaw value from hardware, without any zeroing/offset, in degrees. Positive is
   * CounterClockWise.
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
    gyro.isConnected();

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
}
