package ca.team1310.swerve.core.config;

/**
 * Configuration for the encoder.
 *
 * @param type the type of encoder, either Cancoder or Integrated encoder
 * @param inverted true if the encoder is upside down with respect to its expected orientation
 * @param retrySeconds how long to wait before retrying communication with the encoder
 * @param retryCount how many times to retry communication with the encoder
 */
public record EncoderConfig(
    EncoderType type, boolean inverted, double retrySeconds, int retryCount) {

  public static EncoderConfig cancoder(boolean inverted, double retrySeconds, int retryCount) {
    return new EncoderConfig(EncoderType.CANCODER, inverted, retrySeconds, retryCount);
  }

  public static EncoderConfig integrated(boolean inverted) {
    return new EncoderConfig(EncoderType.INTEGRATED, inverted, 0, 0);
  }
}
