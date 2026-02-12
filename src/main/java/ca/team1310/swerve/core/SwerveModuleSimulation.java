package ca.team1310.swerve.core;

import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.utils.Coordinates;
import edu.wpi.first.wpilibj.Timer;

class SwerveModuleSimulation implements SwerveModule {

  private final String name;
  private final Coordinates location;
  private final Timer timer = new Timer();
  private double dt;
  private double lastTime;
  private final ModuleState currentState;
  private final double maxAttainableModuleSpeedMps;

  public SwerveModuleSimulation(ModuleConfig cfg, double maxAttainableModuleSpeedMps) {
    this.name = cfg.name();
    this.location = cfg.location();
    this.currentState = new ModuleState();
    this.currentState.setLocation(cfg.location());
    this.timer.start();
    this.lastTime = this.timer.get();
    this.dt = 0.0;
    this.maxAttainableModuleSpeedMps = maxAttainableModuleSpeedMps;
  }

  @Override
  public String getName() {
    return this.name;
  }

  @Override
  public Coordinates getLocation() {
    return location;
  }

  @Override
  public void readState() {}

  @Override
  public void syncEncoders() {}

  @Override
  public void readVerboseState() {}

  @Override
  public ModuleState getState() {
    return this.currentState;
  }

  @Override
  public void setDesiredState(ModuleDirective desiredState) {
    double now = timer.get();
    this.dt = now - this.lastTime;
    this.lastTime = now;

    this.currentState.setDesiredAngle(desiredState.getAngle());
    this.currentState.setDesiredSpeed(desiredState.getSpeed());
    this.currentState.setAngle(desiredState.getAngle());
    this.currentState.setVelocity(desiredState.getSpeed());
    this.currentState.setPosition(currentState.getPosition() + desiredState.getSpeed() * this.dt);
    this.currentState.setAbsoluteEncoderAngle(desiredState.getAngle());
    this.currentState.setDriveOutputPower(desiredState.getSpeed() / maxAttainableModuleSpeedMps);
  }

  /**
   * Check for active faults, manage alerts for them, and return status.
   *
   * @return true if there are active faults
   */
  public boolean checkFaults() {
    return false;
  }
}
