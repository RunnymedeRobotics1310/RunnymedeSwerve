package ca.team1310.swervedrive.core.hardware.neosparkmax;

import ca.team1310.swervedrive.core.AngleMotor;
import ca.team1310.swervedrive.core.config.MotorConfig;
import com.revrobotics.CANSparkBase;

public class NSMAngleMotor extends NSMMotor implements AngleMotor {

    public NSMAngleMotor(int canId, MotorConfig cfg) {
        super(canId);
        // instantiate & configure motor
        this.motor.setInverted(cfg.inverted());
        configureSparkMax(() -> motor.enableVoltageCompensation(cfg.nominalVoltage()));
        configureSparkMax(() -> motor.setSmartCurrentLimit(cfg.currentLimitAmps()));
        configureSparkMax(() -> motor.setClosedLoopRampRate(cfg.rampRateSecondsZeroToFull()));

        // configure integrated encoder
        final double angleConversionFactor = 360 / cfg.gearRatio();
        // report in degrees not rotations
        configureSparkMax(() -> encoder.setPositionConversionFactor(angleConversionFactor));
        // report in degrees per second not rotations per minute
        configureSparkMax(() -> encoder.setVelocityConversionFactor(angleConversionFactor / 60));

        pid.setFeedbackDevice(encoder);
        configureSparkMax(() -> pid.setP(cfg.p(), 0));
        configureSparkMax(() -> pid.setI(cfg.i(), 0));
        configureSparkMax(() -> pid.setD(cfg.d(), 0));
        configureSparkMax(() -> pid.setFF(cfg.ff(), 0));
        configureSparkMax(() -> pid.setIZone(cfg.izone(), 0));
        configureSparkMax(() -> pid.setOutputRange(-180, 180, 0));
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
        configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(-180));
        configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(180));

        setMotorBrake(true);

        burnFlash();
    }

    @Override
    public double getPosition() {
        return (encoder.getPosition() + 360) % 360;
    }

    @Override
    public void setReferenceAngle(double degrees) {
        configureSparkMax(() -> pid.setReference(degrees, CANSparkBase.ControlType.kPosition, 0, 0));
    }

    @Override
    public void setEncoderPosition(double actualAngleDegrees) {
        if (encoder.getPosition() != actualAngleDegrees) {
            configureSparkMax(() -> encoder.setPosition(actualAngleDegrees));
        }
    }
}
