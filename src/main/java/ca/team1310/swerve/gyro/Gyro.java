package ca.team1310.swerve.gyro;

/**
 * Represents a gyro that can be used to determine the orientation of the robot.
 */
public interface Gyro {
    /**
     * Reset pitch, yaw, and roll to 0 degrees.
     */
    void zeroGyro();

    /**
     * Get the roll of the robot, in degrees.
     * @return the roll of the robot, in degrees
     */
    double getRoll();

    /**
     * Get the pitch of the robot, in degrees.
     * @return the pitch of the robot, in degrees
     */
    double getPitch();

    /**
     * Get the yaw of the robot, in degrees.
     * @return the yaw of the robot, in degrees
     */
    double getYaw();

    /**
     * Get the rate of yaw change of the robot, in degrees.
     * @return the rate of rotation of the yaw of the robot, in degrees per second
     */
    double getYawRate();
}
