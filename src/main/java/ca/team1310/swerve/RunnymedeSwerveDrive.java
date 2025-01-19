package ca.team1310.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
     * The main periodic method for the swerve drive. This should be called at the very top
     * of the periodic method of any subsystem using this swerve drive.
     * <p>
     * This method is responsible reading key sensors and updating the state of the swerve drive.
     */
    void periodic();

    /**
     * The main internal method for controlling the drivebase. This code does not apply any
     * limiters or validation, and should be used by implementing swerve drive subsystems
     * only.
     * <p>
     * Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
     *
     * @param rawDesiredRobotOrientedVelocity The intended velocity of the robot chassis relative to
     *                                        itself.
     */
    void drive(ChassisSpeeds rawDesiredRobotOrientedVelocity);

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
     * <strong>This is a backdoor function. Use with caution.</strong>
     * <p>
     * This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY
     * when testing parts of the drivebase in a controlled environment.
     *
     * @param moduleName   the module to activate
     * @param desiredState the state of the specified module.
     */
    void setModuleState(String moduleName, SwerveModuleState desiredState);

    /**
     * Change the robot's internal understanding of its position and rotation. This
     * is not an incremental change or suggestion, it discontinuously re-sets the
     * pose to the specified pose.
     *
     * @param pose the new location and heading of the robot.
     */
    void resetOdometry(Pose2d pose);

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    Pose2d getPose();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    void zeroGyro();

    /**
     * Get the current roll of the robot, in degrees, directly from the gyro.
     * @return the roll of the robot, in degrees
     */
    double getGyroRoll();

    /**
     * Get the current pitch of the robot, in degrees, directly from the gyro.
     * @return the pitch of the robot, in degrees
     */
    double getGyroPitch();

    /**
     * Get the current yaw of the robot, in degrees, directly from the gyro.
     * This may, possibly, differ from the rotation returned from <code>getPose()</code>
     * via odometry. Most of the time they will align.
     * @return the yaw of the robot, in degrees
     */
    double getGyroYaw();
}
