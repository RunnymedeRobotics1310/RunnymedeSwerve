package ca.team1310.swerve.core.hardware.neosparkmax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;


class NSMMotor {
    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int maximumRetries = 5;

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    NSMMotor(int canBusId) {
        // instantiate & configure motor
        this.motor = new SparkMax(canBusId, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.controller = this.motor.getClosedLoopController();
        doWithRetry(motor::clearFaults);
    }

    protected final void doWithRetry(Supplier<REVLibError> sparkMaxOperation) {
        for (int i = 0; i < maximumRetries; i++) {
            if (sparkMaxOperation.get() == REVLibError.kOk) {
                return;
            }
            Timer.delay(Units.Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure communicating with motor " + motor.getDeviceId(), true);
    }
}