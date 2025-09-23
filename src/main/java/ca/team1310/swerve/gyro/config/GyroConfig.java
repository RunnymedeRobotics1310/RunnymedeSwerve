package ca.team1310.swerve.gyro.config;

/**
 * Configuration for the gyro used by the swerve drive.
 *
 * @param type the type of gyro to use (NavX or Pigeon2)
 * @param canBusId the CAN bus ID for Pigeon2 (ignored for NavX)
 * @param onlyYaw whether to only use yaw data for Pigeon2 (ignored for NavX)
 * @author Jeff Zakrzewski
 * @since 2025-09-10
 */
public record GyroConfig(GyroType type, int canBusId, boolean onlyYaw) {

  /**
   * Create a NavX gyro configuration.
   *
   * @return a GyroConfig for NavX
   */
  public static GyroConfig navx() {
    return new GyroConfig(GyroType.NAVX, 0, false);
  }

  /**
   * Create a Pigeon2 gyro configuration with default settings.
   *
   * @param canBusId the CAN bus ID for the Pigeon2
   * @return a GyroConfig for Pigeon2
   */
  public static GyroConfig pigeon2(int canBusId) {
    return new GyroConfig(GyroType.PIGEON2, canBusId, true);
  }

  /**
   * Create a Pigeon2 gyro configuration with custom settings.
   *
   * @param canBusId the CAN bus ID for the Pigeon2
   * @param onlyYaw whether to only use yaw data (saves CAN bus utilization)
   * @return a GyroConfig for Pigeon2
   */
  public static GyroConfig pigeon2(int canBusId, boolean onlyYaw) {
    return new GyroConfig(GyroType.PIGEON2, canBusId, onlyYaw);
  }
}
