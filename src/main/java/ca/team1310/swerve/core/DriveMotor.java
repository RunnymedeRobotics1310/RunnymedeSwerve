package ca.team1310.swerve.core;

/**
 * An interface over a drive motor for this swerve drive system. Simplifies interactions between the motor and the module.
 */
public interface DriveMotor {
    /**
     * Get the distance the motor has traveled since the last reset in metres
     * @return the distance the motor has traveled since the last reset in metres
     */
    double getDistance();

    /**
     * Set the target velocity in metres per second
     *
     * @param targetVelocityMPS the desired reference velocity in metres per second
     */
    void setReferenceVelocity(double targetVelocityMPS);

    /**
     * Get the velocity of the motor in metres per second
     * @return the velocity of the motor in metres per second
     */
    double getVelocity();
}
