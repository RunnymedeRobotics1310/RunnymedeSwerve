package ca.team1310.swerve.gyro;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

/** Represents a gyro that can be used to determine the orientation of the robot. */
public interface Gyro extends NTSendable {
  /** Reset pitch, yaw, and roll to 0 degrees. */
  void zeroGyro();

  /**
   * Set the yaw offset of the robot, in degrees. This is useful for setting the robot's initial
   * orientation.
   *
   * @param yaw the yaw offset of the robot, in degrees
   */
  void setYaw(double yaw);

  /**
   * Get the roll of the robot, in degrees.
   *
   * @return the roll of the robot, in degrees
   */
  double getRoll();

  /**
   * Get the pitch of the robot, in degrees.
   *
   * @return the pitch of the robot, in degrees
   */
  double getPitch();

  /**
   * Get the yaw of the robot, in degrees. Positive is CounterClockWise.
   *
   * @return the yaw of the robot, in degrees
   */
  double getYaw();

  /**
   * Get the rate of yaw change of the robot, in degrees.
   *
   * @return the rate of rotation of the yaw of the robot, in degrees per second
   */
  double getYawRate();

  @Override
  default void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty(
        "Value",
        () -> {
          double angle = getYaw();

          // Round the angle to 2 decimal places for the Dashboard
          return Math.round(angle * 100d) / 100d;
        },
        null);
  }
}
