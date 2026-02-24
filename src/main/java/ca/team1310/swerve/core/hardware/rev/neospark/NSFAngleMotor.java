package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.EncoderConfig;
import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

/**
 * @author Tony Field
 * @since 2025-01-26 07:17
 */
public class NSFAngleMotor extends NSAngleMotor<SparkFlex> {

  /**
   * Construct a properly configured angle motor.
   *
   * @param canId The CAN ID of the motor
   * @param cfg The configuration of the motor
   * @param encoderConfig the configuration of the absolute encoder
   */
  public NSFAngleMotor(int canId, MotorConfig cfg, EncoderConfig encoderConfig) {
    super(new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless), cfg, encoderConfig);
  }
}
