package ca.team1310.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** This marks a subsystem as a RunnymedeSwerveSubsystem and exposes the RunnymedeSwerveDrive. */
public interface RunnymedeSwerveSubsystem extends Subsystem {

  /**
   * Get a hold of the RunnymedeSwerveDrive. This is used by the calibration system and should not
   * be used in any other way.
   *
   * @return the RunnymedeSwerveDrive instance.
   */
  public RunnymedeSwerveDrive getRunnymedeSwerveDrive();
}
