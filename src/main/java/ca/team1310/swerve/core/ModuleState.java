package ca.team1310.swerve.core;

import ca.team1310.swerve.utils.Coordinates;

/**
 * @author Tony Field
 * @since 2025-02-01 19:19
 */
public class ModuleState {

  private Coordinates location;

  private double drivePosition,
      driveVelocity,
      driveOutputPower,
      anglePosition,
      absoluteEncoderAngle,
      desiredSpeed,
      desiredAngle = 0;

  ModuleState() {}

  /**
   * Get the location of the module, in metres from the center of the robot
   *
   * @return location
   */
  public Coordinates getLocation() {
    return location;
  }

  void setLocation(Coordinates location) {
    this.location = location;
  }

  /**
   * Get the drive position of the module
   *
   * @return position of the drive motor in metres
   */
  public double getPosition() {
    return drivePosition;
  }

  void setPosition(double drivePosition) {
    this.drivePosition = drivePosition;
  }

  void setVelocity(double driveVelocity) {
    this.driveVelocity = driveVelocity;
  }

  /**
   * Get the power output of the drive motor
   *
   * @return power output of the drive motor in the range -1.0 to 1.0
   */
  public double getDriveOutputPower() {
    return driveOutputPower;
  }

  void setDriveOutputPower(double driveOutputPower) {
    this.driveOutputPower = driveOutputPower;
  }

  void setAngle(double anglePosition) {
    this.anglePosition = anglePosition;
  }

  /**
   * Get the absolute encoder angle of the module
   *
   * @return the absolute encoder angle of the module in degrees
   */
  public double getAbsoluteEncoderAngle() {
    return absoluteEncoderAngle;
  }

  void setAbsoluteEncoderAngle(double absoluteEncoderAngle) {
    this.absoluteEncoderAngle = absoluteEncoderAngle;
  }

  /**
   * Get the angle of the module
   *
   * @return the angle of the module in degrees from -180 to 180 (ccw positive)
   */
  public double getAngle() {
    return anglePosition;
  }

  /**
   * Get the speed of the module
   *
   * @return the speed of the degrees in m/s
   */
  public double getSpeed() {
    return driveVelocity;
  }

  /**
   * Get the desired speed of the module
   *
   * @return speed in m/s
   */
  public double getDesiredSpeed() {
    return desiredSpeed;
  }

  void setDesiredSpeed(double desiredSpeed) {
    this.desiredSpeed = desiredSpeed;
  }

  /**
   * Get the desired angle of the module
   *
   * @return angle in degrees
   */
  public double getDesiredAngle() {
    return desiredAngle;
  }

  void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }
}
