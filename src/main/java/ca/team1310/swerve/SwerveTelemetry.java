package ca.team1310.swerve;

import ca.team1310.swerve.vision.PoseConfidence;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to hold telemetry data for a swerve drive. All fields are public to facilitate rapid access and measurement tracking
 */
public final class SwerveTelemetry {

    /**
     * The prefix to use for all SmartDashboard keys
     */
    public static final String PREFIX = "1310/";

    private final int moduleCount;

    // Core
    /**
     * The maximum speed of swerve modules in metres per second
     */
    public double maxModuleSpeedMPS = Double.MIN_VALUE;
    /**
     * The maximum translational speed of the robot in metres per second
     */
    public double maxTranslationSpeedMPS = Double.MIN_VALUE;
    /**
     * The maximum rotational velocity of the robot in radians per second
     */
    public double maxRotationalVelocityRadPS = Double.MIN_VALUE;
    /**
     * The track width of the robot in metres
     */
    public double trackWidthMetres = Double.MIN_VALUE;
    /**
     * The wheel base of the robot in metres
     */
    public double wheelBaseMetres = Double.MIN_VALUE;
    /**
     * The radius of the swerve module wheels in metres
     */
    public double wheelRadiusMetres = Double.MIN_VALUE;
    /**
     * The desired chassis speeds of the robot in metres per second and radians per second. [0] is x, [1] is y, [2] is omega
     */
    public double[] desiredChassisSpeeds = new double[3];
    /**
     * The measured chassis speeds of the robot in metres per second and radians per second. [0] is x, [1] is y, [2] is omega
     */
    public double[] measuredChassisSpeeds = new double[3];

    // Module
    /**
     * The names of the swerve modules
     */
    public String[] moduleNames;
    /**
     * The locations of the swerve module wheels in metres relative to the center of the robot.
     */
    public double[] moduleWheelLocations;
    /**
     * The measured states of the swerve modules in metres per second and radians per second.
     */
    public double[] moduleMeasuredStates;
    /**
     * The desired states of the swerve modules in metres per second and radians per second.
     */
    public double[] moduleDesiredStates;
    /**
     * The absolute encoder offsets of the swerve modules in degrees
     */
    public double[] angleEncoderAbsoluteOffsetDegrees;
    /**
     * The absolute encoder positions of the swerve modules in degrees
     */
    public double[] moduleAbsoluteEncoderPositionDegrees;
    /**
     * The angle motor positions of the swerve modules in degrees
     */
    public double[] moduleAngleMotorPositionDegrees;
    /**
     * The drive motor positions of the swerve modules in metres
     */
    public double[] moduleDriveMotorPositionMetres;

    // Gyro
    /**
     * The raw yaw of the gyro in degrees
     */
    public double gyroRawYawDegrees = Double.MIN_VALUE;
    /**
     * The adjusted yaw of the gyro in degrees
     */
    public double gyroAdjustedYawDegrees = Double.MIN_VALUE;
    /**
     * The raw pitch of the gyro in degrees
     */
    public double gyroRawPitchDegrees = Double.MIN_VALUE;
    /**
     * The raw roll of the gyro in degrees
     */
    public double gyroRawRollDegrees = Double.MIN_VALUE;

    // Pose
    /**
     * The x position of the robot with respect to the field in metres
     */
    public double poseMetresX = Double.MIN_VALUE;
    /**
     * The y position of the robot with respect to the field in metres
     */
    public double poseMetresY = Double.MIN_VALUE;
    /**
     * The heading of the robot with respect to the field in degrees
     */
    public double poseHeadingDegrees = Double.MIN_VALUE;

    // Vision
    /**
     * Whether the vision system has updated the pose of the robot
     */
    public boolean visionPoseUpdate = false;
    /**
     * The confidence of the last vision measurement of the robot
     */
    public PoseConfidence visionPoseConfidence = PoseConfidence.NONE;
    /**
     * The priority id from the vision system
     */
    public double visionPriorityId = Double.MIN_VALUE;
    /**
     * The tid value from the vision system
     */
    public double visionTid = Double.MIN_VALUE;
    /**
     * The tx value from the vision system
     */
    public double visionTx = Double.MIN_VALUE;
    /**
     * The ty value from the vision system
     */
    public double visionTy = Double.MIN_VALUE;
    /**
     * The ta value from the vision system
     */
    public double visionTa = Double.MIN_VALUE;
    /**
     * The tl value from the vision system
     */
    public double visionTl = Double.MIN_VALUE;
    /**
     * The x position of the robot with respect to the field as measured by the vision system in metres
     */
    public double visionPoseX = Double.MIN_VALUE;
    /**
     * The y position of the robot with respect to the field as measured by the vision system in metres
     */
    public double visionPoseY = Double.MIN_VALUE;
    /**
     * The heading of the robot with respect to the field as measured by the vision system in degrees
     */
    public double visionPoseHeading = Double.MIN_VALUE;
    /**
     * The average distance to the vision targets in metres
     */
    public double visionTargetAvgDist = Double.MIN_VALUE;
    /**
     * The number of april tags detected
     */
    public int visionNumTags = Integer.MIN_VALUE;
    /**
     * The detailed information detected from the april tags
     */
    public String visionAprilTagInfo = "";
    /**
     * The difference between the vision pose and the swerve pose in metres
     */
    public double visionPoseSwerveDiff = Double.MIN_VALUE;

    // Field Oriented
    /**
     * The x velocity of the robot with respect to the field in metres per second
     */
    public double fieldOrientedVelocityX = Double.MIN_VALUE;
    /**
     * The y velocity of the robot with respect to the field in metres per second
     */
    public double fieldOrientedVelocityY = Double.MIN_VALUE;
    /**
     * The rotational velocity of the robot with respect to the field in radians per second
     */
    public double fieldOrientedVelocityOmega = Double.MIN_VALUE;
    /**
     * The x distance in metres from the current robot location to the desired pose. Not set by RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
     */
    public double fieldOrientedDeltaToPoseX = Double.MIN_VALUE;
    /**
     * The y distance in metres from the current robot location to the desired pose. Not set by RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
     */
    public double fieldOrientedDeltaToPoseY = Double.MIN_VALUE;
    /**
     * The heading in degrees from the current robot location to the desired pose. Not set by RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
     */
    public double fieldOrientedDeltaToPoseHeading = Double.MIN_VALUE;

    /**
     * Whether the swerve advantage scope constants have been posted to SmartDashboard
     */
    private boolean advantageScopeConstantsPosted = false;

    /**
     * Create and initialize a new swerve telemetry object
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
    }

    /**
     * Post all telemetry data to SmartDashboard
     */
    public void post() {
        postSwerveAdvantageScopeConstants();
        postSwerveAdvantageScope();
        postRunnymedeSwerveTelemetry();
        postYagslExtensions();
    }

    private void postSwerveAdvantageScopeConstants() {
        if (!advantageScopeConstantsPosted && moduleWheelLocations[0] != 0.0) {
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

    private void postSwerveAdvantageScope() {
        SmartDashboard.putNumberArray("swerve/measuredStates", moduleMeasuredStates);
        SmartDashboard.putNumberArray("swerve/desiredStates", moduleDesiredStates);
        SmartDashboard.putNumber("swerve/robotRotation", poseHeadingDegrees);
        SmartDashboard.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
        SmartDashboard.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);
    }

    private void postYagslExtensions() {
        if (moduleNames[0] == null) {
            return;
        }
        SmartDashboard.putString(
            "RobotVelocity",
            String.format(
                "%.2f m/s, %.2f m/s @ %.2f rad/s",
                desiredChassisSpeeds[0],
                desiredChassisSpeeds[1],
                desiredChassisSpeeds[2]
            )
        );
        SmartDashboard.putNumber("Raw IMU Yaw", gyroRawYawDegrees);
        SmartDashboard.putNumber("Adjusted IMU Yaw", gyroAdjustedYawDegrees);

        for (int i = 0; i < moduleCount; i++) {
            //            String pfx = PREFIX + "Swerve/Module["+moduleNames[i];
            String pfx = "Module[" + moduleNames[i];
            //            SmartDashboard.putNumber(pfx + "]/ SysId Drive Power", );
            //            SmartDashboard.putNumber(pfx + "]/ SysId Drive Position", );
            //            SmartDashboard.putNumber(pfx + "]/ SysId Drive Velocity", );
            //            SmartDashboard.putNumber(pfx + "]/ SysId Angle Power", );
            //            SmartDashboard.putNumber(pfx + "]/ SysId Angle Position", );
            //            SmartDashboard.putNumber(pfx + "]/ SysId Absolute Encoder Velocity", );
            SmartDashboard.putNumber(pfx + "]/ Angle Setpoint", moduleDesiredStates[i * 2]);
            SmartDashboard.putNumber(pfx + "]/ Speed Setpoint", moduleDesiredStates[i * 2 + 1]);

            SmartDashboard.putNumber(
                pfx + "]/ Raw Absolute Encoder",
                moduleAbsoluteEncoderPositionDegrees[i] + angleEncoderAbsoluteOffsetDegrees[i]
            );
            SmartDashboard.putNumber(pfx + "]/ Raw Angle Encoder", moduleAngleMotorPositionDegrees[i]);
            SmartDashboard.putNumber(pfx + "]/ Raw Drive Encoder", moduleDriveMotorPositionMetres[i]);
            SmartDashboard.putNumber(pfx + "]/ Adjusted Absolute Encoder", moduleAbsoluteEncoderPositionDegrees[i]);
            //            SmartDashboard.putNumber(pfx + "]/ absoluteEncoderIssueName", "" );
        }
    }

    private void postRunnymedeSwerveTelemetry() {
        SmartDashboard.putString(PREFIX + "Swerve/gyroHeading", String.format("%.1f deg", gyroRawYawDegrees));

        double vX = desiredChassisSpeeds[0];
        double vY = desiredChassisSpeeds[1];
        double speed = Math.hypot(vX, vY);
        double omega = desiredChassisSpeeds[2];
        String vRobot = String.format("%.1f (%.1f, %.1f) m/s %.1f deg/s", speed, vX, vY, omega);
        SmartDashboard.putString(PREFIX + "Swerve/velocity_robot", vRobot);

        double fieldSpeed = Math.hypot(fieldOrientedVelocityX, fieldOrientedVelocityY);
        String vField = String.format(
            "%.1f (%.1f, %.1f) m/s %.1f deg/s",
            fieldSpeed,
            fieldOrientedVelocityX,
            fieldOrientedVelocityY,
            fieldOrientedVelocityOmega
        );
        SmartDashboard.putString(PREFIX + "Swerve/velocity_field", vField);

        String poseOdo = String.format("(%.2f, %.2f) m %.1f deg", poseMetresX, poseMetresY, poseHeadingDegrees);
        SmartDashboard.putString(PREFIX + "Swerve/pose_odo", poseOdo);

        String poseVis = String.format("(%.2f, %.2f) m %.1f deg", visionPoseX, visionPoseY, visionPoseHeading);
        SmartDashboard.putString(PREFIX + "Swerve/pose_vis", poseVis);

        String delta = String.format(
            "(%.2f, %.2f) m %.1f deg",
            fieldOrientedDeltaToPoseX,
            fieldOrientedDeltaToPoseY,
            fieldOrientedDeltaToPoseHeading
        );
        SmartDashboard.putString(PREFIX + "Swerve/distance_to_pose", delta);
    }
}
