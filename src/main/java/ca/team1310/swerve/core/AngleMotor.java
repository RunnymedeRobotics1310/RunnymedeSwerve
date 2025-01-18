package ca.team1310.swerve.core;

/**
 * Interface for an angle motor. It exposes the methods that are necessary for an angle motor that the module requires,
 * and hides implementation details.
 */
public interface AngleMotor {
    /**
     * Called at the start of each robot period. Used to read sensor values
     */
    void periodic();

    /**
     * Get the current position of the angle motor in degrees
     * @return the current position of the motor in degrees
     */
    double getPosition();

    /**
     * Set the target angle of the motor
     *
     * @param degrees the desired reference angle in degrees
     */
    void setReferenceAngle(double degrees);

    /**
     * Reset the internal encoder of this motor to the current position
     * @param actualAngleDegrees the current position of the motor in degrees
     */
    void setEncoderPosition(double actualAngleDegrees);
}
