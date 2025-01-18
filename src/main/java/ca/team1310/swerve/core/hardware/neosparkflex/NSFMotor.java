package ca.team1310.swerve.core.hardware.neosparkflex;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

/**
 * Represents a Neo motor controlled by a SparkFlex.
 */
class NSFMotor {

    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int maximumRetries = 5;

    /**
     * The SparkFlex motor controller that is used to interact with the motor.
     */
    protected final SparkFlex sparkFlexMotorController;
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
    NSFMotor(int canBusId) {
        // instantiate & configure motor
        this.sparkFlexMotorController = new SparkFlex(canBusId, MotorType.kBrushless);
        this.encoder = this.sparkFlexMotorController.getEncoder();
        this.controller = this.sparkFlexMotorController.getClosedLoopController();
        doWithRetry(sparkFlexMotorController::clearFaults);
    }

    /**
     * Perform an operation on the SparkFlex, and retry it if there is a failure.
     * @param sparkFlexOperation The operation to perform
     */
    protected final void doWithRetry(Supplier<REVLibError> sparkFlexOperation) {
        for (int i = 0; i < maximumRetries; i++) {
            if (sparkFlexOperation.get() == REVLibError.kOk) {
                return;
            }
            Timer.delay(Units.Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure communicating with motor " + sparkFlexMotorController.getDeviceId(), true);
    }
}
