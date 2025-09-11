package ca.team1310.swerve.gyro.hardware;

import static ca.team1310.swerve.utils.SwerveUtils.normalizeDegrees;

import ca.team1310.swerve.gyro.Gyro;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;

/**
 * @author Jeff Zakrzewski
 * @since 2025-09-10
 *     <p>A gyro that uses the Pigeon2 to get the robot's orientation.
 */
public class Pigeon2 implements Gyro {

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;

  private final boolean onlyYaw;

  private double rollOffset;
  private double pitchOffset;
  private double yawOffset;

  private final Alert pigeonNotPresent =
      new Alert("Pigeon2 Not Detected.  Odometry will be wrong!", Alert.AlertType.kError);

  /** Create a new Pigeon2 gyro with specified CAN bus ID */
  public Pigeon2(int canBusId, boolean onlyYaw) {
    this.onlyYaw = onlyYaw;

    com.ctre.phoenix6.hardware.Pigeon2 pigeon = new com.ctre.phoenix6.hardware.Pigeon2(canBusId);

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    this.yaw = pigeon.getYaw();
    this.yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(100);
    yawVelocity.setUpdateFrequency(50.0);

    // Not setting these saves CAN Bus utilization, so don't turn them on if we don't need them
    if (!onlyYaw) {
      this.roll = pigeon.getRoll();
      this.pitch = pigeon.getPitch();

      roll.setUpdateFrequency(100);
      pitch.setUpdateFrequency(100);
    } else {
      this.roll = null;
      this.pitch = null;
    }

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void setYaw(double yaw) {
    yawOffset = this.yaw.getValueAsDouble() - yaw;
  }

  @Override
  public void zeroGyro() {
    rollOffset = roll.getValueAsDouble();
    pitchOffset = pitch.getValueAsDouble();
    yawOffset = yaw.getValueAsDouble();
  }

  @Override
  public double getRoll() {
    return onlyYaw ? 0 : roll.getValueAsDouble() - rollOffset;
  }

  @Override
  public double getPitch() {
    return onlyYaw ? 0 : pitch.getValueAsDouble() - pitchOffset;
  }

  @Override
  public double getYaw() {
    return normalizeDegrees(yaw.getValueAsDouble() - yawOffset);
  }

  public double getYawRaw() {
    return normalizeDegrees(yaw.getValueAsDouble());
  }

  @Override
  public double getYawRate() {
    return yawVelocity.getValueAsDouble();
  }

  public boolean isConnected() {
    boolean connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    pigeonNotPresent.set(!connected);
    return connected;
  }
}
