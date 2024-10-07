package ca.team1310.swervedrive;

import ca.team1310.swervedrive.vision.PoseConfidence;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SwerveTelemetry {

    public static final String PREFIX                          = "1310/";

    private final int          moduleCount;

    // Core
    public double              maxModuleSpeedMPS               = Double.MIN_VALUE;
    public double              maxTranslationSpeedMPS          = Double.MIN_VALUE;
    public double              maxRotationalVelocityRadPS      = Double.MIN_VALUE;
    public double              trackWidthMetres                = Double.MIN_VALUE;
    public double              wheelBaseMetres                 = Double.MIN_VALUE;
    public double              wheelRadiusMetres               = Double.MIN_VALUE;
    public double[]            desiredChassisSpeeds            = new double[3];
    public double[]            measuredChassisSpeeds           = new double[3];

    // Module
    public String[]            moduleNames;
    public double[]            moduleWheelLocations;

    public double[]            moduleMeasuredStates;

    public double[]            moduleDesiredStates;
    public double[]            angleEncoderAbsoluteOffsetDegrees;
    public double[]            moduleAbsoluteEncoderPositionDegrees;
    public double[]            moduleAngleMotorPositionDegrees;
    public double[]            moduleDriveMotorPositionMetres;

    // Gyro
    public double              gyroRawYawDegrees               = Double.MIN_VALUE;
    public double              gyroAdjustedYawDegrees          = Double.MIN_VALUE;
    public double              gyroRawPitchDegrees             = Double.MIN_VALUE;
    public double              gyroRawRollDegrees              = Double.MIN_VALUE;

    // Pose
    public double              poseMetresX                     = Double.MIN_VALUE;
    public double              poseMetresY                     = Double.MIN_VALUE;
    public double              poseHeadingDegrees              = Double.MIN_VALUE;

    // Vision
    public boolean             visionPoseUpdate                = false;
    public PoseConfidence      visionPoseConfidence            = PoseConfidence.NONE;
    public double              visionPriorityId                = Double.MIN_VALUE;
    public double              visionTid                       = Double.MIN_VALUE;
    public double              visionTx                        = Double.MIN_VALUE;
    public double              visionTy                        = Double.MIN_VALUE;
    public double              visionTa                        = Double.MIN_VALUE;
    public double              visionTl                        = Double.MIN_VALUE;
    public double              visionPoseX                     = Double.MIN_VALUE;
    public double              visionPoseY                     = Double.MIN_VALUE;
    public double              visionPoseHeading               = Double.MIN_VALUE;
    public double              visionTargetAvgDist             = Double.MIN_VALUE;
    public int                 visionNumTags                   = Integer.MIN_VALUE;
    public String              visionAprilTagInfo              = "";
    public double              visionPoseSwerveDiff            = Double.MIN_VALUE;

    // Field Oriented
    public double              fieldOrientedVelocityX          = Double.MIN_VALUE;
    public double              fieldOrientedVelocityY          = Double.MIN_VALUE;
    public double              fieldOrientedVelocityOmega      = Double.MIN_VALUE;
    public double              fieldOrientedDeltaToPoseX       = Double.MIN_VALUE;
    public double              fieldOrientedDeltaToPoseY       = Double.MIN_VALUE;
    public double              fieldOrientedDeltaToPoseHeading = Double.MIN_VALUE;


    private boolean            advantageScopeConstantsPosted   = false;

    public SwerveTelemetry(int moduleCount) {
        this.moduleCount                     = moduleCount;
        moduleNames                          = new String[moduleCount];
        moduleWheelLocations                 = new double[moduleCount * 2];
        moduleDesiredStates                  = new double[moduleCount * 2];
        angleEncoderAbsoluteOffsetDegrees    = new double[moduleCount];
        moduleMeasuredStates                 = new double[moduleCount * 2];
        moduleAbsoluteEncoderPositionDegrees = new double[moduleCount];
        moduleAngleMotorPositionDegrees      = new double[moduleCount];
        moduleDriveMotorPositionMetres       = new double[moduleCount];
    }

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
        SmartDashboard.putString("RobotVelocity", String.format("%.2f m/s, %.2f m/s @ %.2f rad/s",
            desiredChassisSpeeds[0], desiredChassisSpeeds[1], desiredChassisSpeeds[2]));
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

            SmartDashboard.putNumber(pfx + "]/ Raw Absolute Encoder",
                moduleAbsoluteEncoderPositionDegrees[i] + angleEncoderAbsoluteOffsetDegrees[i]);
            SmartDashboard.putNumber(pfx + "]/ Raw Angle Encoder", moduleAngleMotorPositionDegrees[i]);
            SmartDashboard.putNumber(pfx + "]/ Raw Drive Encoder", moduleDriveMotorPositionMetres[i]);
            SmartDashboard.putNumber(pfx + "]/ Adjusted Absolute Encoder", moduleAbsoluteEncoderPositionDegrees[i]);
//            SmartDashboard.putNumber(pfx + "]/ absoluteEncoderIssueName", "" );
        }
    }

    private void postRunnymedeSwerveTelemetry() {
        SmartDashboard.putString(PREFIX + "Swerve/gyroHeading", String.format("%.1f deg", gyroRawYawDegrees));

        double vX     = desiredChassisSpeeds[0];
        double vY     = desiredChassisSpeeds[1];
        double speed  = Math.hypot(vX, vY);
        double omega  = desiredChassisSpeeds[2];
        String vRobot = String.format("%.1f (%.1f, %.1f) m/s %.1f deg/s", speed, vX, vY, omega);
        SmartDashboard.putString(PREFIX + "Swerve/velocity_robot", vRobot);

        double fieldSpeed = Math.hypot(fieldOrientedVelocityX, fieldOrientedVelocityY);
        String vField     = String.format("%.1f (%.1f, %.1f) m/s %.1f deg/s",
            fieldSpeed, fieldOrientedVelocityX, fieldOrientedVelocityY, fieldOrientedVelocityOmega);
        SmartDashboard.putString(PREFIX + "Swerve/velocity_field", vField);

        String poseOdo = String.format("(%.2f, %.2f) m %.1f deg", poseMetresX, poseMetresY, poseHeadingDegrees);
        SmartDashboard.putString(PREFIX + "Swerve/pose_odo", poseOdo);

        String poseVis = String.format("(%.2f, %.2f) m %.1f deg", visionPoseX, visionPoseY, visionPoseHeading);
        SmartDashboard.putString(PREFIX + "Swerve/pose_vis", poseVis);

        String delta = String.format("(%.2f, %.2f) m %.1f deg", fieldOrientedDeltaToPoseX, fieldOrientedDeltaToPoseY,
            fieldOrientedDeltaToPoseHeading);
        SmartDashboard.putString(PREFIX + "Swerve/distance_to_pose", delta);

    }
}
