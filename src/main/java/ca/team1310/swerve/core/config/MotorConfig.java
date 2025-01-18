package ca.team1310.swerve.core.config;

/**
 * Represents the general configuration of a motor.
 * @param inverted Whether the motor is inverted
 * @param currentLimitAmps The maximum current that the motor can draw
 * @param nominalVoltage The nominal voltage of the motor
 * @param rampRateSecondsZeroToFull The time it takes for the motor to go from 0 to full power. The controller will honour this.
 * @param gearRatio The gear ratio of the motor to the wheel
 * @param p The proportional gain of the motor
 * @param i The integral gain of the motor
 * @param d The derivative gain of the motor
 * @param ff The feedforward gain of the motor
 * @param izone The integral zone of the motor
 */
public record MotorConfig(
    MotorType type,
    boolean inverted,
    int currentLimitAmps,
    double nominalVoltage,
    double rampRateSecondsZeroToFull,
    double gearRatio,
    double p,
    double i,
    double d,
    double ff,
    double izone
) {}
