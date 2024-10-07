package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.SwerveTelemetry;
import ca.team1310.swervedrive.core.config.ModuleConfig;
import ca.team1310.swervedrive.core.hardware.cancoder.CanCoder;
import ca.team1310.swervedrive.core.hardware.neosparkmax.NSMAngleMotor;
import ca.team1310.swervedrive.core.hardware.neosparkmax.NSMDriveMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveModuleImpl implements SwerveModule {

    private final String               name;
    private final Translation2d        location;
    private final DriveMotor           driveMotor;
    private final AngleMotor           angleMotor;
    private final AbsoluteAngleEncoder angleEncoder;
    private final int                  internalEncoderUpdateFrequency;
    private int                        internalEncoderUpdateCount = 0;
    private SwerveModuleState          desiredState;


    SwerveModuleImpl(ModuleConfig cfg) {
        this.name                           = cfg.name();
        this.location                       = new Translation2d(cfg.xPositionMetres(), cfg.yPositionMetres());
        this.driveMotor                     = new NSMDriveMotor(cfg.driveMotorCanId(), cfg.driveMotorConfig(),
            cfg.wheelRadiusMetres());
        this.angleMotor                     = new NSMAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig());
        this.angleEncoder                   = new CanCoder(cfg.angleEncoderCanId(), cfg.angleEncoderAbsoluteOffsetDegrees(),
            cfg.absoluteAngleEncoderConfig());
        this.internalEncoderUpdateFrequency = cfg.angleMotorEncoderUpdateFrequency();
    }

    public String getName() {
        return name;
    }

    public Translation2d getLocation() {
        return location;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getDistance(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        updateMotors();
        updateInternalEncoder();
    }

    private void updateMotors() {

        Rotation2d        currentHeading = Rotation2d.fromDegrees(angleMotor.getPosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentHeading);

        /*
         * Scale down speed when wheels aren't facing the right direction
         *
         * If the angle error is close to 0 degrees, we are aligned properly, so we can apply
         * full power to drive wheels. If the angle error is close to 90 degrees, driving in
         * any direction does not help.
         *
         * Scale speed by cosine of angle error. This scales down movement perpendicular to the
         * desired direction of travel that can occur when modules change directions. This results
         * in smoother driving.
         */
        Rotation2d        steerError     = optimizedState.angle.minus(currentHeading);
        double            cosineScalar   = steerError.getCos();
        optimizedState.speedMetersPerSecond *= (cosineScalar < 0 ? 0 : cosineScalar);


        driveMotor.setReferenceVelocity(optimizedState.speedMetersPerSecond);

        angleMotor.setReferenceAngle(optimizedState.angle.getDegrees());
    }

    private void updateInternalEncoder() {
        if (internalEncoderUpdateCount++ >= internalEncoderUpdateFrequency) {
            internalEncoderUpdateCount = 0;
            double angle = angleEncoder.getPosition();
            if (angle >= 0) {
                angleMotor.setEncoderPosition(angle);
            }
        }
    }

    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        // identify the module
        telemetry.moduleNames[moduleIndex]                     = name;
        telemetry.moduleWheelLocations[moduleIndex * 2]        = location.getX();
        telemetry.moduleWheelLocations[moduleIndex * 2 + 1]    = location.getY();

        // desired states
        telemetry.moduleDesiredStates[moduleIndex * 2]         = desiredState.angle.getDegrees();
        telemetry.moduleDesiredStates[moduleIndex * 2 + 1]     = desiredState.speedMetersPerSecond;

        // measured states
        telemetry.moduleMeasuredStates[moduleIndex * 2]        = angleMotor.getPosition();
        telemetry.moduleMeasuredStates[moduleIndex * 2 + 1]    = driveMotor.getVelocity();

        // position information
        telemetry.moduleAngleMotorPositionDegrees[moduleIndex] = angleMotor.getPosition();
        telemetry.moduleDriveMotorPositionMetres[moduleIndex]  = driveMotor.getDistance();

        angleEncoder.populateTelemetry(telemetry, moduleIndex);
    }
}
