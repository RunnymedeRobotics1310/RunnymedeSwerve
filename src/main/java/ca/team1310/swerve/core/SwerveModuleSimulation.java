package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.ModuleConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

class SwerveModuleSimulation implements SwerveModule {

    private final String name;
    private final double locationOnRobotX;
    private final double locationOnRobotY;
    private final Timer timer = new Timer();
    private double dt;
    private double fakePos;
    private double fakeSpeed;
    private double lastTime;
    private ModuleState currentState;
    private ModuleState desiredState;
    private ModulePosition modulePosition;

    public SwerveModuleSimulation(ModuleConfig cfg) {
        this.name = cfg.name();
        this.locationOnRobotX = cfg.xPositionMetres();
        this.locationOnRobotY = cfg.yPositionMetres();
        this.timer.start();
        this.lastTime = this.timer.get();
        this.currentState = new ModuleState();
        this.desiredState = new ModuleState();
        this.modulePosition = new ModulePosition();
        this.fakeSpeed = 0.0;
        this.fakePos = 0.0;
        this.dt = 0.0;
    }

    @Override
    public String getName() {
        return this.name;
    }

    @Override
    public ModulePosition getPosition() {
        modulePosition.setDistance(this.fakePos);
        modulePosition.setAngle(this.currentState.getAngle());
        return modulePosition;
    }

    @Override
    public ModuleState getState() {
        return this.currentState;
    }

    @Override
    public void setDesiredState(ModuleState desiredState) {
        this.desiredState = desiredState;
        this.dt = this.timer.get() - this.lastTime;
        this.lastTime = this.timer.get();
        this.currentState = desiredState;
        this.fakeSpeed = desiredState.getSpeed();
        this.fakePos += this.fakeSpeed * this.dt;
    }

    @Override
    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        if (telemetry.enabled) {
            // identify the module
            telemetry.moduleNames[moduleIndex] = name;
            telemetry.moduleWheelLocations[moduleIndex * 2] = locationOnRobotX;
            telemetry.moduleWheelLocations[moduleIndex * 2 + 1] = locationOnRobotY;

            // desired states
            telemetry.moduleDesiredStates[moduleIndex * 2] = desiredState.getAngle();
            telemetry.moduleDesiredStates[moduleIndex * 2 + 1] = desiredState.getSpeed();

            // measured states
            double fakeAngle = desiredState.getAngle();
            telemetry.moduleMeasuredStates[moduleIndex * 2] = fakeAngle;
            telemetry.moduleMeasuredStates[moduleIndex * 2 + 1] = fakeSpeed;

            // position information
            telemetry.moduleAbsoluteEncoderPositionDegrees[moduleIndex] = fakeAngle;
            telemetry.moduleAngleMotorPositionDegrees[moduleIndex] = fakeAngle;
            telemetry.moduleDriveMotorPositionMetres[moduleIndex] = fakePos;
        }
    }
}
