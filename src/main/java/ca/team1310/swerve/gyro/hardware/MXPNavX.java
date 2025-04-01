package ca.team1310.swerve.gyro.hardware;

import static ca.team1310.swerve.utils.SwerveUtils.normalizeDegrees;

import ca.team1310.swerve.gyro.Gyro;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.AHRS;

/** A gyro that uses the NavX MXP to get the robot's orientation. */
public class MXPNavX implements Gyro {

  private final AHRS navx;

  private double rollOffset;
  private double pitchOffset;
  private double yawOffset;

  private final Alert navxNotPresent =
      new Alert("NaxV Not Detected.  Odometry will be wrong!", Alert.AlertType.kError);

  /** Create a new MXPNavX gyro */
  public MXPNavX() {
    this.navx = new AHRS(SPI.Port.kMXP, (byte) 100);
    this.rollOffset = navx.getRoll();

    this.pitchOffset = navx.getPitch();

    this.yawOffset = -navx.getYaw();
  }

  @Override
  public void setYaw(double yaw) {
    yawOffset = (-navx.getYaw()) - yaw;
  }

  @Override
  public void zeroGyro() {
    rollOffset = navx.getRoll();
    pitchOffset = navx.getPitch();
    yawOffset = -navx.getYaw();
  }

  @Override
  public double getRoll() {
    return navx.getRoll() - rollOffset;
  }

  @Override
  public double getPitch() {
    return navx.getPitch() - pitchOffset;
  }

  @Override
  public double getYaw() {
    return normalizeDegrees((-navx.getYaw()) - yawOffset);
  }

  public double getYawRaw() {
    return normalizeDegrees(-navx.getYaw());
  }

  @Override
  public double getYawRate() {
    return navx.getRate();
  }

  public boolean isConnected() {
    boolean connected = navx.isConnected();
    navxNotPresent.set(!connected);
    return connected;
  }
}
