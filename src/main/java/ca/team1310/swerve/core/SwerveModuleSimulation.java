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
    private ModuleState currentState;

    public SwerveModuleSimulation(ModuleConfig cfg) {
        this.name = cfg.name();
        this.location = new Coordinates(cfg.xPositionMetres(), cfg.yPositionMetres());
        this.timer.start();
        this.lastTime = this.timer.get();
        this.currentState = new ModuleState();
        this.dt = 0.0;
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
    public void updateState(boolean odometry, boolean vision, boolean telemetry) {}

    @Override
    public ModuleState getState() {
        return this.currentState;
    }

    @Override
    public void setDesiredState(ModuleDirective desiredState) {
        this.dt = this.timer.get() - this.lastTime;
        this.lastTime = this.timer.get();

        this.currentState.setDesiredAngle(desiredState.getAngle());
        this.currentState.setDesiredSpeed(desiredState.getSpeed());
        this.currentState.setAngle(desiredState.getAngle());
        this.currentState.setVelocity(desiredState.getSpeed());
        this.currentState.setPosition(currentState.getPosition() + desiredState.getSpeed() * this.dt);
        this.currentState.setAbsoluteEncoderAngle(desiredState.getAngle());
        this.currentState.setAngleVelocity(0);
        this.currentState.setDriveOutputPower(0);
    }
}
