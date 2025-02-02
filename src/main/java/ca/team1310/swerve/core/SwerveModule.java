package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;

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
     * Get the current swerve module position state. This includes the "distance" and "angle" of the module.
     * @return the current position of the module
     */
    ModulePosition getPosition();

    /**
     * Set the desired state of the swerve module. This includes the "speed" and "angle" of the module.
     * @param desiredState the desired state of the module
     */
    void setDesiredState(ModuleState desiredState);

    /**
     * Get the current swerve module state. This includes the "speed" and "angle" of the module.
     * @return the current state of the module
     */
    ModuleState getState();

    /**
     * Populate the telemetry object with the module's data.
     * @param telemetry the telemetry object to populate
     * @param moduleIndex the index of the module in the swerve drive
     */
    void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex);
}
