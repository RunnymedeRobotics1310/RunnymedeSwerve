package ca.team1310.swerve;

import ca.team1310.swerve.core.ModuleState;
import ca.team1310.swerve.odometry.FieldPose;

/**
 * A Swerve Drive system that can be easily used in a robot's drive subsystem. Configuration is explicitly specified
 * via code, and this interface describes all of the ways the main robot can interact with the swerve drive.  More
 * importantly, it describes all the ways the main robot NEEDS to interact with the swerve drive.
 * <p>
 * It's recognized that the codebase required to operate a swerve drive can be quite complex, and this interface
 * and library exist to provide a simple and easy-to-use interface for teams to use.  At the same time, advanced
 * high school students should be able to fully understand what is going on inside this swerve drive codebase and
 * enhance it as required.
 */
public interface RunnymedeSwerveDrive {
    /**
     * The main internal method for controlling the drivebase. This code does not apply any
     * limiters or validation, and should be used by implementing swerve drive subsystems
     * only.
     * <p>
     * Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
     *
     * @param vx The desired velocity of the robot in the x direction in meters per second.
     *           Positive is forward.
     * @param vy The desired velocity of the robot in the y direction in meters per second.
     *           Positive is to the left.
     * @param omega The desired angular velocity of the robot in radians per second.
     *              Positive is counter-clockwise.
     */
    void drive(double vx, double vy, double omega);

    /**
     * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
     * nearly stationary.
     *
     * @return true if successfully locked, false otherwise
     */
    boolean lock();

    /**
     * Set the desired module state for the named module WHEN IN TEST MODE ONLY.
     * <p>
     * <strong>This is a backdoor function. Use with caution.</strong>
     * <p>
     * This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY
     * when testing parts of the drivebase in a controlled environment.
     *
     * @param moduleName   the module to activate
     * @param speed - the speed of the drive motor in m/s
     *             @param angle - the angle of the wheel in degrees
     */
    void setModuleState(String moduleName, double speed, double angle);

    /**
     * Change the robot's internal understanding of its position and rotation. This
     * is not an incremental change or suggestion, it discontinuously re-sets the
     * pose to the specified pose.
     *
     * @param pose the new location and heading of the robot.
     */
    void resetOdometry(FieldPose pose);

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    FieldPose getPose();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    void zeroGyro();

    /**
     * Get the current roll of the robot, in degrees, directly from the gyro.
     * @return the roll of the robot, in degrees
     */
    double getRoll();

    /**
     * Get the current pitch of the robot, in degrees, directly from the gyro.
     * @return the pitch of the robot, in degrees
     */
    double getPitch();

    /**
     * Get the current yaw of the robot, in degrees, directly from the gyro.
     * This may, possibly, differ from the rotation returned from <code>getPose()</code>
     * via odometry. Most of the time they will align.
     * @return the yaw of the robot, in degrees
     */
    double getYaw();

    /**
     * Update the telemetry of the swerve drive, using data from the drivebase.
     * <p>
     * This function can be called at whatever period is desired. Data returned
     * is not guaranteed to be all set at the exact same time - some values
     * may be updated more frequently than others. The data update frequencies
     * are controlled via configuration.
     *
     * @param telemetry the telemetry object to update
     */
    void updateTelemetry(SwerveTelemetry telemetry);
}
