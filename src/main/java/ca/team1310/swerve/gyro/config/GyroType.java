package ca.team1310.swerve.gyro.config;

/**
 * Enum representing the type of gyro to use for the swerve drive.
 *
 * @author Jeff Zakrzewski
 * @since 2025-09-10
 */
public enum GyroType {
  /** Use NavX MXP gyro connected via SPI */
  NAVX,
  /** Use Pigeon2 gyro connected via CAN */
  PIGEON2
}
