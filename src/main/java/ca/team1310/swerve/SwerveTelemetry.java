package ca.team1310.swerve;

import static ca.team1310.swerve.core.config.TelemetryLevel.*;

import ca.team1310.swerve.core.config.TelemetryLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to hold telemetry data for a swerve drive. All fields are public to facilitate rapid
 * access and measurement tracking
 */
public final class SwerveTelemetry {

  /** The desired telemetry level. */
  public TelemetryLevel level = TelemetryLevel.NONE;

  /** The prefix to use for all SmartDashboard keys */
  public static final String PREFIX = "1310/";

  private final int moduleCount;

  // Core
  /** The maximum speed of swerve modules in metres per second */
  public double maxModuleSpeedMPS = Double.MIN_VALUE;

  /** The maximum translational speed of the robot in metres per second */
  public double maxTranslationSpeedMPS = Double.MIN_VALUE;

  /** The maximum rotational velocity of the robot in radians per second */
  public double maxRotationalVelocityRadPS = Double.MIN_VALUE;

  /** The track width of the robot in metres */
  public double trackWidthMetres = Double.MIN_VALUE;

  /** The wheel base of the robot in metres */
  public double wheelBaseMetres = Double.MIN_VALUE;

  /** The radius of the swerve module wheels in metres */
  public double wheelRadiusMetres = Double.MIN_VALUE;

  /**
   * The desired chassis speeds of the robot in metres per second and radians per second. [0] is x,
   * [1] is y, [2] is omega
   */
  public double[] desiredChassisSpeeds = new double[3];

  /**
   * The measured chassis speeds of the robot in metres per second and radians per second. [0] is x,
   * [1] is y, [2] is omega
   */
  public double[] measuredChassisSpeeds = new double[3];

  // Module
  /** The names of the swerve modules */
  public String[] moduleNames;

  /** The locations of the swerve module wheels in metres relative to the center of the robot. */
  public double[] moduleWheelLocations;

  /** The measured states of the swerve modules in metres per second and radians per second. */
  public double[] moduleMeasuredStates;

  /** The desired states of the swerve modules in metres per second and radians per second. */
  public double[] moduleDesiredStates;

  /** The absolute encoder offsets of the swerve modules in degrees */
  public double[] angleEncoderAbsoluteOffsetDegrees;

  /** The absolute encoder positions of the swerve modules in degrees */
  public double[] moduleAbsoluteEncoderPositionDegrees;

  /** The angle motor positions of the swerve modules in degrees */
  public double[] moduleAngleMotorPositionDegrees;

  /** The drive motor positions of the swerve modules in metres */
  public double[] moduleDriveMotorPositionMetres;

  /** The drive motor output power of the swerve modules */
  public double[] driveMotorOutputPower;

  // Pose
  /** The x location of the robot with respect to the field in metres */
  public double poseMetresX = Double.MIN_VALUE;

  /** The y location of the robot with respect to the field in metres */
  public double poseMetresY = Double.MIN_VALUE;

  /** The heading of the robot with respect to the field in degrees */
  public double poseHeadingDegrees = Double.MIN_VALUE;

  /**
   * The x location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseX = Double.MIN_VALUE;

  /**
   * The y location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseY = Double.MIN_VALUE;

  /**
   * The heading of the robot with respect to the field as measured by the vision system in degrees
   */
  public double visionPoseHeading = Double.MIN_VALUE;

  /** Whether the swerve advantage scope constants have been posted to SmartDashboard */
  private boolean advantageScopeConstantsPosted = false;

  public boolean hasVisPose = false;

  /**
   * Create and initialize a new swerve telemetry object
   *
   * @param moduleCount the number of swerve modules
   */
  public SwerveTelemetry(int moduleCount) {
    this.moduleCount = moduleCount;
    moduleNames = new String[moduleCount];
    moduleWheelLocations = new double[moduleCount * 2];
    moduleDesiredStates = new double[moduleCount * 2];
    angleEncoderAbsoluteOffsetDegrees = new double[moduleCount];
    moduleMeasuredStates = new double[moduleCount * 2];
    moduleAbsoluteEncoderPositionDegrees = new double[moduleCount];
    moduleAngleMotorPositionDegrees = new double[moduleCount];
    moduleDriveMotorPositionMetres = new double[moduleCount];
    driveMotorOutputPower = new double[moduleCount];
  }

  /** Post all telemetry data to SmartDashboard */
  public void post() {
    SmartDashboard.putBoolean("swerve/hasVisPose", hasVisPose);

    if (level == INPUT || level == CALCULATED || level == VERBOSE) {
      postConstants();
      postInput();
    }

    if (level == CALCULATED || level == VERBOSE) {
      postConstants();
      postInput();
      postCalculated();
    }
    if (level == VERBOSE) {
      postConstants();
      postInput();
      postCalculated();
      postVerbose();
    }
  }

  private void postConstants() {
    if (!advantageScopeConstantsPosted && moduleWheelLocations[0] != 0.0) {
      // advantagescope constants
      SmartDashboard.putNumber("swerve/moduleCount", moduleCount);
      SmartDashboard.putNumberArray("swerve/wheelLocations", moduleWheelLocations);
      SmartDashboard.putNumber("swerve/maxSpeed", maxTranslationSpeedMPS);
      SmartDashboard.putString("swerve/rotationUnit", "degrees");
      SmartDashboard.putNumber("swerve/sizeLeftRight", trackWidthMetres);
      SmartDashboard.putNumber("swerve/sizeFrontBack", wheelBaseMetres);
      SmartDashboard.putString("swerve/forwardDirection", "up");
      SmartDashboard.putNumber("swerve/maxAngularVelocity", maxRotationalVelocityRadPS);
      advantageScopeConstantsPosted = true;
    }
  }

  private void postInput() {
    // advantagescope
    SmartDashboard.putNumberArray("swerve/desiredStates", moduleDesiredStates);
    SmartDashboard.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);
    // other inputs
  }

  private void postCalculated() {
    // advantagescope
    SmartDashboard.putNumberArray("swerve/measuredStates", moduleMeasuredStates);
    SmartDashboard.putNumber("swerve/robotRotation", poseHeadingDegrees);
    SmartDashboard.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
    // other calculated values
  }

  private void postVerbose() {
    double vX = desiredChassisSpeeds[0];
    double vY = desiredChassisSpeeds[1];
    double speed = Math.hypot(vX, vY);
    double omega = desiredChassisSpeeds[2];
    String vRobot = String.format("%.2f (%.2f, %.2f) m/s %.1f deg/s", speed, vX, vY, omega);
    SmartDashboard.putString(PREFIX + "Swerve/velocity_desired_robot", vRobot);

    vX = measuredChassisSpeeds[0];
    vY = measuredChassisSpeeds[1];
    speed = Math.hypot(vX, vY);
    omega = measuredChassisSpeeds[2];
    vRobot = String.format("%.2f (%.2f, %.2f) m/s %.1f deg/s", speed, vX, vY, omega);
    SmartDashboard.putString(PREFIX + "Swerve/velocity_measured_robot", vRobot);

    String poseOdo =
        String.format("(%.2f, %.2f) m %.1f deg", poseMetresX, poseMetresY, poseHeadingDegrees);
    SmartDashboard.putString(PREFIX + "Swerve/pose_odo", poseOdo);

    String poseVis =
        String.format("(%.2f, %.2f) m %.1f deg", visionPoseX, visionPoseY, visionPoseHeading);
    SmartDashboard.putString(PREFIX + "Swerve/pose_vis", poseVis);

    for (int i = 0; i < moduleCount; i++) {
      String name = moduleNames[i];
      double pwr = driveMotorOutputPower[i];
      SmartDashboard.putString(PREFIX + "Swerve/drive_power_" + name, String.format("%.3f", pwr));
    }
  }
}
