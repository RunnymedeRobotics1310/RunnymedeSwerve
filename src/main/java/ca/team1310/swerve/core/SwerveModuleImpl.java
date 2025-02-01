package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.core.hardware.cancoder.CanCoder;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFDriveMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMDriveMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

class SwerveModuleImpl implements SwerveModule {

    private static final double ANGLEENCODERPERIODSECONDS = 1.0;

    private final String name;
    private final Translation2d location;
    private final DriveMotor driveMotor;
    private final AngleMotor angleMotor;
    private final AbsoluteAngleEncoder angleEncoder;
    private SwerveModuleState desiredState;
    private Notifier encoderSynchronizer = new Notifier(this::syncAngleEncoder);
    private final Lock encoderLock = new ReentrantLock();

    SwerveModuleImpl(ModuleConfig cfg, int robotPeriodMillis) {
        this.name = cfg.name();
        this.location = new Translation2d(cfg.xPositionMetres(), cfg.yPositionMetres());
        this.driveMotor = getDriveMotor(cfg, robotPeriodMillis);
        this.angleMotor = getAngleMotor(cfg, robotPeriodMillis);
        this.angleEncoder = getAbsoluteAngleEncoder(cfg);
        this.encoderSynchronizer.startPeriodic(ANGLEENCODERPERIODSECONDS);
        this.encoderSynchronizer.setName("RunnymedeSwerve Angle Encoder Sync " + name);
    }

    private DriveMotor getDriveMotor(ModuleConfig cfg, int robotPeriodMillis) {
        return switch (cfg.driveMotorConfig().type()) {
            case NEO_SPARK_FLEX -> new NSFDriveMotor(
                cfg.driveMotorCanId(),
                cfg.driveMotorConfig(),
                cfg.wheelRadiusMetres(),
                robotPeriodMillis
            );
            default -> new NSMDriveMotor(
                cfg.driveMotorCanId(),
                cfg.driveMotorConfig(),
                cfg.wheelRadiusMetres(),
                robotPeriodMillis
            );
        };
    }

    private AngleMotor getAngleMotor(ModuleConfig cfg, int robotPeriodMillis) {
        return switch (cfg.angleMotorConfig().type()) {
            case NEO_SPARK_FLEX -> new NSFAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig(), robotPeriodMillis);
            default -> new NSMAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig(), robotPeriodMillis);
        };
    }

    private AbsoluteAngleEncoder getAbsoluteAngleEncoder(ModuleConfig cfg) {
        return new CanCoder(
            cfg.angleEncoderCanId(),
            cfg.angleEncoderAbsoluteOffsetDegrees(),
            cfg.absoluteAngleEncoderConfig()
        );
    }

    private void syncAngleEncoder() {
        encoderLock.lock();
        try {
            angleMotor.setEncoderPosition(angleMotor.getPosition());
        } finally {
            encoderLock.unlock();
        }
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
    }

    private void updateMotors() {
        Rotation2d currentHeading = Rotation2d.fromDegrees(angleMotor.getPosition());

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(currentHeading);

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
        Rotation2d steerError = desiredState.angle.minus(currentHeading);
        double cosineScalar = steerError.getCos();
        desiredState.speedMetersPerSecond *= (cosineScalar < 0 ? 0 : cosineScalar);

        driveMotor.setReferenceVelocity(desiredState.speedMetersPerSecond);

        angleMotor.setReferenceAngle(desiredState.angle.getDegrees());
    }

    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        if (telemetry.enabled) {
            // identify the module
            telemetry.moduleNames[moduleIndex] = name;
            telemetry.moduleWheelLocations[moduleIndex * 2] = location.getX();
            telemetry.moduleWheelLocations[moduleIndex * 2 + 1] = location.getY();

            // desired states
            if (desiredState != null) {
                telemetry.moduleDesiredStates[moduleIndex * 2] = desiredState.angle.getDegrees();
                telemetry.moduleDesiredStates[moduleIndex * 2 + 1] = desiredState.speedMetersPerSecond;
            }

            // measured states
            telemetry.moduleMeasuredStates[moduleIndex * 2] = angleMotor.getPosition();
            telemetry.moduleMeasuredStates[moduleIndex * 2 + 1] = driveMotor.getVelocity();

            // position information
            telemetry.moduleAngleMotorPositionDegrees[moduleIndex] = angleMotor.getPosition();
            telemetry.moduleDriveMotorPositionMetres[moduleIndex] = driveMotor.getDistance();
            telemetry.driveMotorOutputPower[moduleIndex] = driveMotor.getMeasuredVoltage();

            angleEncoder.populateTelemetry(telemetry, moduleIndex);
        }
    }
}
