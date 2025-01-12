package ca.team1310.swerve.core.hardware.neosparkmax;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * Represents a Neo motor controlled by a SparkMax.
 */
class NSMMotor {

    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int maximumRetries = 5;

    /**
     * The SparkMax motor controller that is used to interact with the motor.
     */
    protected final SparkMax sparkMaxMotorController;
    /**
     * The relative encoder that is used to measure the motor's position and velocity.
     */
    protected final RelativeEncoder encoder;
    /**
     * The closed loop controller that is used to control the motor.
     */
    protected final SparkClosedLoopController controller;

    /**
     * Construct a properly configured motor.
     * @param canBusId The CAN ID of the motor
     */
    NSMMotor(int canBusId) {
        // instantiate & configure motor
        this.sparkMaxMotorController = new SparkMax(canBusId, MotorType.kBrushless);
        this.encoder = this.sparkMaxMotorController.getEncoder();
        this.controller = this.sparkMaxMotorController.getClosedLoopController();
        doWithRetry(sparkMaxMotorController::clearFaults);
    }

    /**
     * Perform an operation on the SparkMax, and retry it if there is a failure.
     * @param sparkMaxOperation The operation to perform
     */
    protected final void doWithRetry(Supplier<REVLibError> sparkMaxOperation) {
        for (int i = 0; i < maximumRetries; i++) {
            if (sparkMaxOperation.get() == REVLibError.kOk) {
                return;
            }
            Timer.delay(Units.Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure communicating with motor " + sparkMaxMotorController.getDeviceId(), true);
    }
}
