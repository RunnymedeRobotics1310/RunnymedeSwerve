package ca.team1310.swervedrive.core.hardware.neosparkmax;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

class NSMMotor {
    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int                  maximumRetries = 5;
    protected final CANSparkMax        motor;
    protected final RelativeEncoder    encoder;
    protected final SparkPIDController pid;

    NSMMotor(int canBusId) {
        // instantiate & configure motor
        this.motor   = new CANSparkMax(canBusId, CANSparkLowLevel.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        pid          = motor.getPIDController();
        configureCANStatusFrames(10, 20, 20, 500, 500);
        configureSparkMax(motor::restoreFactoryDefaults);
        configureSparkMax(motor::clearFaults);
    }

    /**
     * Run the configuration until it succeeds or times out.
     *
     * @param config Lambda supplier returning the error state.
     */
    protected final void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
    }

    protected final void setMotorBrake(boolean isBrakeMode) {
        configureSparkMax(() -> motor.setIdleMode(isBrakeMode ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast));
    }

    protected final void burnFlash() {
        try {
            Thread.sleep(200);
        }
        catch (Exception e) {
        }
        configureSparkMax(() -> motor.burnFlash());
    }

    /**
     * Set the CAN status frames.
     *
     * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
     * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
     * @param CANStatus2 Motor Position
     * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
     * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
     */
    private void configureCANStatusFrames(int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
        configureSparkMax(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, CANStatus0));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, CANStatus1));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, CANStatus2));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, CANStatus3));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, CANStatus4));
        // TODO: Configure Status Frame 5 and 6 if necessary
        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
    }
}
