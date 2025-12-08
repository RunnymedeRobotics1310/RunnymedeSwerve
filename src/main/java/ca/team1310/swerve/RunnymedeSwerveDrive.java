package ca.team1310.swerve;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A Swerve Drive system that can be easily used in a robot's drive subsystem. Configuration is
 * explicitly specified via code, and this interface describes all of the ways the main robot can
 * interact with the swerve drive. More importantly, it describes all the ways the main robot NEEDS
 * to interact with the swerve drive.
 *
 * <p>It's recognized that the codebase required to operate a swerve drive can be quite complex, and
 * this interface and library exist to provide a simple and easy-to-use interface for teams to use.
 * At the same time, advanced high school students should be able to fully understand what is going
 * on inside this swerve drive codebase and enhance it as required.
 */
public interface RunnymedeSwerveDrive {
  /**
   * The main internal method for controlling the drivebase. This code does not apply any limiters
   * or validation, and should be used by implementing swerve drive subsystems only.
   *
   * <p>Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
   *
   * @param vx The desired velocity of the robot in the x direction in meters per second. Positive
   *     is forward.
   * @param vy The desired velocity of the robot in the y direction in meters per second. Positive
   *     is to the left.
   * @param omega The desired angular velocity of the robot in radians per second. Positive is
   *     counter-clockwise.
   */
  void driveRobotOriented(double vx, double vy, double omega);

  /**
   * The other main internal method for controlling the drivebase. This code does not apply any
   * limiters or validation, and should be used by implementing swerve drive subsystems only.
   *
   * <p>Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
   *
   * @param vx The desired field oriented velocity of the robot in the x direction in meters per
   *     second. Positive is towards the red alliance.
   * @param vy The desired field oriented velocity of the robot in the y direction in meters per
   *     second. Positive away from the scoring table.
   * @param omega The desired angular velocity of the robot in radians per second. Positive is
   *     counter-clockwise.
   */
  default void driveFieldOriented(double vx, double vy, double omega) {}

  /**
   * Obtain the robot relative velocity vX, vY, vR
   *
   * @return A double[] with {vX, vY, vR}
   */
  double[] getMeasuredRobotVelocity();

  /**
   * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
   * nearly stationary.
   *
   * @return true if successfully locked, false otherwise
   */
  boolean lock();

  /**
   * Set the desired module state for the named module WHEN IN TEST MODE ONLY.
   *
   * <p><strong>This is a backdoor function. Use with caution.</strong>
   *
   * <p>This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY when
   * testing parts of the drivebase in a controlled environment.
   *
   * @param moduleName the module to activate
   * @param speed - the speed of the drive motor in m/s
   * @param angle - the angle of the wheel in degrees
   */
  void setModuleState(String moduleName, double speed, double angle);

  /**
   * Change the robot's internal understanding of its location and rotation. This is not an
   * incremental change or suggestion, it discontinuously re-sets the pose to the specified pose.
   *
   * <p>The default implementation does nothing.
   *
   * @param pose the new location and heading of the robot.
   */
  default void resetOdometry(Pose2d pose) {}

  /**
   * Gets the current pose (location and rotation) of the robot, as reported by odometry.
   *
   * <p>The default implementation returns a pose at 0,0,0.
   *
   * @return The robot's pose
   */
  default Pose2d getPose() {
    return new Pose2d();
  }

  /**
   * Set the gyro yaw offset of the robot, in degrees.
   *
   * @param yaw the yaw offset of the robot, in degrees
   */
  default void setYaw(double yaw) {}

  /**
   * Resets the gyro angle to zero and resets odometry to the same location, but facing toward 0.
   *
   * <p>The default implementation does nothing.
   */
  default void zeroGyro() {}

  /**
   * Get the current roll of the robot, in degrees, directly from the gyro.
   *
   * <p>The default implementation returns 0.
   *
   * @return the roll of the robot, in degrees
   */
  default double getRoll() {
    return 0;
  }

  /**
   * Get the current pitch of the robot, in degrees, directly from the gyro.
   *
   * <p>The default implementation returns 0.
   *
   * @return the pitch of the robot, in degrees
   */
  default double getPitch() {
    return 0;
  }

  /**
   * Get the current yaw of the robot, in degrees, from the gyro with zeroing/offset applied. This
   * may, possibly, differ from the rotation returned from <code>getPose()</code> via odometry. Most
   * of the time they will align.
   *
   * <p>The default implementation returns 0.
   *
   * @return the yaw of the robot, in degrees
   */
  default double getYaw() {
    return 0;
  }

  /**
   * Get the rate of change of the yaw of the robot in degrees per second.
   *
   * <p>The default implementation returns 0.
   *
   * @return the yaw rate of the robot, in degrees per second
   */
  default double getYawRate() {
    return 0;
  }
}
