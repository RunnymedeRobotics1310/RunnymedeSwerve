package ca.team1310.swerve.core;

/**
 * Interface for an angle motor. It exposes the methods that are necessary for an angle motor that
 * the module requires, and hides implementation details.
 */
public interface AngleMotor {
  /**
   * Get the current location of the angle motor in degrees
   *
   * @return the current location of the motor in degrees (from -180 to +180, CCW positive)
   */
  double getPosition();

  /**
   * Get the rotational velocity of the angle motor in degrees / s
   *
   * @return the angular velocity of the motor
   */
  double getVelocity();

  /**
   * Set the target angle of the motor
   *
   * @param degrees the desired reference angle in degrees, between -180 and 180, CCW positive
   */
  void setReferenceAngle(double degrees);

  /**
   * Reset the internal encoder of this motor to the current location
   *
   * @param actualAngleDegrees the current location of the motor in degrees (from -180 to +180, CCW
   *     positive)
   */
  void setEncoderPosition(double actualAngleDegrees);
}
