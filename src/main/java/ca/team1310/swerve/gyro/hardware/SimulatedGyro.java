package ca.team1310.swerve.gyro.hardware;

import static ca.team1310.swerve.utils.SwerveUtils.normalizeDegrees;

import ca.team1310.swerve.gyro.Gyro;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** A simulated gyro that can be used in simulation mode. */
public class SimulatedGyro implements Gyro {

  private final Timer timer = new Timer();
  private double lastTime;
  private double roll = 0;
  private double pitch = 0;
  private double yaw = 0;
  private double yawRate = 0;

  /** Constructs a new simulated gyro. */
  public SimulatedGyro() {
    this.timer.start();
    this.lastTime = this.timer.get();
  }

  public double getYawRate() {
    return this.yawRate;
  }

  public double getYaw() { return getYawRaw(); }

  public double getYawRaw() {
    return normalizeDegrees(this.yaw);
  }

  public double getPitch() {
    return this.pitch;
  }

  @Override
  public void setYaw(double yaw) { }

  @Override
  public void zeroGyro() {
    this.yaw = 0;
  }

  public double getRoll() {
    return this.roll;
  }

  /**
   * Update the gyro heading with the updated yaw based on the current omega value
   *
   * @param omegaRadiansPerSecond the current omega value in radians per second
   */
  public void updateGyroForSimulation(double omegaRadiansPerSecond) {
    double now = this.timer.get();
    double change = omegaRadiansPerSecond * (now - this.lastTime);
    this.yaw += Units.radiansToDegrees(change);
    this.lastTime = now;
  }
}
