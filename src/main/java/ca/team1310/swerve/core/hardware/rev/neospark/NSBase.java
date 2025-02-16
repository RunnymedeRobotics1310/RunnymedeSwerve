package ca.team1310.swerve.core.hardware.rev.neospark;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * @author Tony Field
 * @since 2025-01-26 06:49
 */
public abstract class NSBase<T extends SparkBase> {

    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int maximumRetries = 5;

    /**
     * The SparkBase motor controller that is used to interact with the motor.
     */
    protected final T spark;
    /**
     * The relative encoder that is used to measure the motor's location and velocity.
     */
    protected final RelativeEncoder encoder;
    /**
     * The closed loop controller that is used to control the motor.
     */
    protected final SparkClosedLoopController controller;

    /**
     * Construct a properly configured motor.
     */
    public NSBase(T spark) {
        // instantiate & configure motor
        this.spark = spark;
        this.encoder = this.spark.getEncoder();
        this.controller = this.spark.getClosedLoopController();
        doWithRetry(spark::clearFaults);
    }

    /**
     * Perform an operation on the Spark motor controller, and retry it if there is a failure.
     * @param sparkOperation The operation to perform
     */
    protected final void doWithRetry(Supplier<REVLibError> sparkOperation) {
        for (int i = 0; i < maximumRetries; i++) {
            if (sparkOperation.get() == REVLibError.kOk) {
                return;
            }
            Timer.delay(Units.Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure communicating with motor " + spark.getDeviceId(), true);
    }
}
