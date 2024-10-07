package ca.team1310.swervedrive.core;

public interface DriveMotor {

    /**
     * Get the distance the motor has traveled since the last reset in metres
     */
    double getDistance();

    /**
     * Set the target velocity in metres per second
     * 
     * @param targetVelocityMPS
     */
    void setReferenceVelocity(double targetVelocityMPS);

    /**
     * Get the velocity of the motor in metres per second
     */
    double getVelocity();
}
