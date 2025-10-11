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

  /** The SparkBase motor controller that is used to interact with the motor. */
  protected final T spark;

  /** The relative encoder that is used to measure the motor's location and velocity. */
  protected final RelativeEncoder encoder;

  /** The closed loop controller that is used to control the motor. */
  protected final SparkClosedLoopController controller;

  /**
   * Construct a properly configured motor.
   *
   * @param spark The spark motor controller
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
   *
   * <p>Per Rev support 2025-10-01: setReference() will continuously send the setpoint to the
   * controller in the background after being called. The period can be adjusted with
   * setControlFramePeriodMs(), and the default value is 20ms. E.g. if the controller fails to
   * receive the initial setpoint command, it will receive it again 20ms later and so on. This
   * ensures the device gets the most up-to-date setpoint as soon as possible and that it is able to
   * recover from events like brownouts. Also, since you mentioned it, in newer versions of REVLib
   * (2025+), there are retry mechanisms and better error handling for failed configurations, so
   * manually retrying a failed config is no longer needed. See setCANMaxRetries() and
   * setCANTimeout().
   *
   * @param sparkOperation The operation to perform
   */
  protected final void doWithRetry(Supplier<REVLibError> sparkOperation) {
    for (int i = 0; i < maximumRetries; i++) {
      if (sparkOperation.get() == REVLibError.kOk) {
        return;
      }
      // on failure sleep 5ms then try again - faster than the default
      Timer.delay(Units.Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure communicating with motor " + spark.getDeviceId(), true);
  }

  /**
   * Are there any active faults on this motor
   *
   * @return true if there are active faults
   */
  public boolean hasFaults() {
    return spark.hasActiveFault();
  }
}
