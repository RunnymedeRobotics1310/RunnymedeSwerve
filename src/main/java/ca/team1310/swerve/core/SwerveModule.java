package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve module that can be controlled and queried for information. The main swerve drive interacts with three or more of these units using only the methods contained herein.
 */
public interface SwerveModule {
    /**
     * Get the name of the swerve module
     * @return the name of the swerve module
     */
    String getName();

    /**
     * Get the location of the swerve module on the robot
     * @return the location of the swerve module on the robot with respect to the robot's center.
     */
    Translation2d getLocation();

    /**
     * Get the current swerve module position state. This includes the "distance" and "angle" of the module.
     * @return the current position of the module
     */
    SwerveModulePosition getPosition();

    /**
     * Get the current swerve module state. This includes the "speed" and "angle" of the module.
     * @return the current state of the module
     */
    SwerveModuleState getState();

    /**
     * Set the desired state of the swerve module. This includes the "speed" and "angle" of the module.
     * @param desiredState the desired state of the module
     */
    void setDesiredState(SwerveModuleState desiredState);

    /**
     * Populate the telemetry object with the module's data.
     * @param telemetry the telemetry object to populate
     * @param moduleIndex the index of the module in the swerve drive
     */
    void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex);
}
