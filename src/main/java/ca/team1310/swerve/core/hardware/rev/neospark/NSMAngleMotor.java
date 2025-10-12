package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

/**
 * @author Tony Field
 * @since 2025-01-26 07:07
 */
public class NSMAngleMotor extends NSAngleMotor<SparkMax> {

  /**
   * Construct a properly configured angle motor.
   *
   * @param canId The CAN ID of the motor
   * @param cfg The configuration of the motor
   */
  public NSMAngleMotor(int canId, MotorConfig cfg) {
    super(new SparkMax(canId, SparkLowLevel.MotorType.kBrushless), cfg);
  }
}
