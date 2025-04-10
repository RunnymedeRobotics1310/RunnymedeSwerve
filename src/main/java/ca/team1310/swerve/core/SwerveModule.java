package ca.team1310.swerve.core;

import ca.team1310.swerve.utils.Coordinates;

/**
 * A swerve module that can be controlled and queried for information. The main swerve drive
 * interacts with three or more of these units using only the methods contained herein.
 */
public interface SwerveModule {
  /**
   * Get the name of the swerve module
   *
   * @return the name of the swerve module
   */
  String getName();

  /**
   * Get the location of the swerve module on the robot. This is the x and y location of the module
   * with respect to the center of the robot, with 0,0 being in the middle and positive x being
   * forward and positive y being right.
   *
   * @return the location of the module with respect to the centre of the robot, in metres.
   */
  Coordinates getLocation();

  /**
   * Set the desired state of the swerve module. This includes the "speed" and "angle" of the
   * module.
   *
   * @param desiredState the desired state of the module
   */
  void setDesiredState(ModuleDirective desiredState);

  /**
   * Update the internal state of the swerve module by reading data from the module hardware. Flags
   * are included to skip some sensor reads. Sensor reads are slow, so this allows some control over
   * which ones are read.
   *
   * @param odometry include necessary data for odometry, including module speed, location, and
   *     angle
   * @param telemetry includes all data
   */
  void readState(boolean odometry, boolean telemetry);

  /**
   * Get the current swerve module state. This can safely be called repeatedly.
   *
   * @return the current state of the module
   */
  ModuleState getState();

  /**
   * Are there any active faults on this motor
   *
   * @return true if there are active faults
   */
  boolean checkFaults();
}
