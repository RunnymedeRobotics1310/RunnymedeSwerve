package ca.team1310.swervedrive.core.hardware.neosparkmax;

import ca.team1310.swervedrive.core.DriveMotor;
import ca.team1310.swervedrive.core.config.MotorConfig;
import com.revrobotics.CANSparkBase;

public class NSMDriveMotor extends NSMMotor implements DriveMotor {
    public NSMDriveMotor(int canId, MotorConfig cfg, double wheelRadiusMetres) {
        super(canId);

        // instantiate & configure motor
        this.motor.setInverted(cfg.inverted());
        configureSparkMax(() -> motor.enableVoltageCompensation(cfg.nominalVoltage()));
        configureSparkMax(() -> motor.setSmartCurrentLimit(cfg.currentLimitAmps()));
        configureSparkMax(() -> motor.setClosedLoopRampRate(cfg.rampRateSecondsZeroToFull()));

        // configure integrated encoder
        final double positionConversionfactor = (2 * Math.PI * wheelRadiusMetres) / cfg.gearRatio();
        // report in metres not rotations
        configureSparkMax(() -> encoder.setPositionConversionFactor(positionConversionfactor));
        // report in metres per second not rotations per minute
        configureSparkMax(() -> encoder.setVelocityConversionFactor(positionConversionfactor / 60));

        pid.setFeedbackDevice(encoder); // Configure feedback of the PID controller as the
        // integrated encoder.
        configureSparkMax(() -> pid.setP(cfg.p(), 0));
        configureSparkMax(() -> pid.setI(cfg.i(), 0));
        configureSparkMax(() -> pid.setD(cfg.d(), 0));
        configureSparkMax(() -> pid.setFF(cfg.ff(), 0));
        configureSparkMax(() -> pid.setIZone(cfg.izone(), 0));
        configureSparkMax(() -> pid.setOutputRange(-1, 1, 0));
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(false));
        setMotorBrake(true);

        burnFlash();
    }

    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public void setReferenceVelocity(double targetVelocityMPS) {
        configureSparkMax(() -> pid.setReference(targetVelocityMPS, CANSparkBase.ControlType.kVelocity, 0, 0));
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
