package ca.team1310.swerve.core;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.ModuleConfig;
import ca.team1310.swerve.core.hardware.cancoder.CanCoder;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSFDriveMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMAngleMotor;
import ca.team1310.swerve.core.hardware.rev.neospark.NSMDriveMotor;
import edu.wpi.first.wpilibj.Notifier;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

class SwerveModuleImpl implements SwerveModule {

    private static final double ANGLEENCODERPERIODSECONDS = 1.0;

    private final String name;
    private final double locationOnRobotX;
    private final double locationOnRobotY;
    private final DriveMotor driveMotor;
    private final AngleMotor angleMotor;
    private final AbsoluteAngleEncoder angleEncoder;
    private ModuleState desiredState = new ModuleState();
    private ModuleState measuredState = new ModuleState();
    private ModulePosition modulePosition = new ModulePosition();
    private final Lock encoderLock = new ReentrantLock();

    SwerveModuleImpl(ModuleConfig cfg, int robotPeriodMillis) {
        this.name = cfg.name();
        this.locationOnRobotX = cfg.xPositionMetres();
        this.locationOnRobotY = cfg.yPositionMetres();
        this.driveMotor = getDriveMotor(cfg, robotPeriodMillis);
        this.angleMotor = getAngleMotor(cfg, robotPeriodMillis);
        this.angleEncoder = getAbsoluteAngleEncoder(cfg);
        Notifier encoderSynchronizer = new Notifier(this::syncAngleEncoder);
        encoderSynchronizer.startPeriodic(ANGLEENCODERPERIODSECONDS);
        encoderSynchronizer.setName("RunnymedeSwerve Angle Encoder Sync " + name);
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

    public ModulePosition getPosition() {
        modulePosition.setDistance(driveMotor.getDistance());
        modulePosition.setAngle(angleMotor.getPosition());
        return modulePosition;
    }

    public ModuleState getState() {
        measuredState.set(driveMotor.getVelocity(), angleMotor.getPosition());
        return measuredState;
    }

    public void setDesiredState(ModuleState desiredState) {
        this.desiredState = desiredState;
        updateMotors();
    }

    private void updateMotors() {
        /*
         * Optimize the reference state to avoid spinning further than 90 degrees
         *
         * Start by figuring out if the module is currently within 90 degrees of the desired angle.
         * THen if it is, flip the heading and reverse the motor
         */
        double currentHeading = angleMotor.getPosition();
        double angleError = Math.abs(desiredState.getAngle() - currentHeading);
        if (angleError > 90 && angleError < 270) {
            if (currentHeading < 0) {
                desiredState.set(-desiredState.getSpeed(), desiredState.getAngle() + 180);
            } else {
                desiredState.set(-desiredState.getSpeed(), desiredState.getAngle() - 180);
            }
        }

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
        angleError = desiredState.getAngle() - currentHeading;
        double cosineScalar = Math.cos(Math.toRadians(angleError));
        desiredState.set(desiredState.getSpeed() * (cosineScalar < 0 ? 0 : cosineScalar), desiredState.getAngle());

        /*
         * Set the reference velocity and angle for the motors
         */
        driveMotor.setReferenceVelocity(desiredState.getSpeed());
        angleMotor.setReferenceAngle(desiredState.getAngle());
    }

    public void populateTelemetry(SwerveTelemetry telemetry, int moduleIndex) {
        if (telemetry.enabled) {
            // identify the module
            telemetry.moduleNames[moduleIndex] = name;
            telemetry.moduleWheelLocations[moduleIndex * 2] = locationOnRobotX;
            telemetry.moduleWheelLocations[moduleIndex * 2 + 1] = locationOnRobotY;

            // desired states
            if (desiredState != null) {
                telemetry.moduleDesiredStates[moduleIndex * 2] = desiredState.getAngle();
                telemetry.moduleDesiredStates[moduleIndex * 2 + 1] = desiredState.getSpeed();
            }

            // measured states
            telemetry.moduleMeasuredStates[moduleIndex * 2] = angleMotor.getPosition();
            telemetry.moduleMeasuredStates[moduleIndex * 2 + 1] = driveMotor.getVelocity();

            // position information
            telemetry.moduleAngleMotorPositionDegrees[moduleIndex] = angleMotor.getPosition();
            telemetry.moduleDriveMotorPositionMetres[moduleIndex] = driveMotor.getDistance();
            telemetry.driveMotorOutputPower[moduleIndex] = driveMotor.getMeasuredVoltage();

            // angle encoder
            telemetry.moduleAbsoluteEncoderPositionDegrees[moduleIndex] = angleEncoder.getPosition();
        }
    }
}
