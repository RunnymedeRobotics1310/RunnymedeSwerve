package ca.team1310.swerve.vision;

import static ca.team1310.swerve.vision.PoseConfidence.NONE;

import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.FieldAwareSwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A field-aware swerve drive that uses a gyro, odometry, and vision to estimate the robot's pose on the field.
 */
public class VisionAwareSwerveDrive extends FieldAwareSwerveDrive {

    // Internal State for handling if we're using MegaTag1 or MegaTag2
    private enum State {
        INITALIZE,
        READY,
    }

    private State state = State.INITALIZE;

    // Internal Limelight/NetworkTable Accessors
    private final NetworkTable table;
    private final DoubleSubscriber txSub;
    private final DoubleSubscriber tySub;
    private final DoubleSubscriber taSub;
    private final DoubleSubscriber tlSub;
    private final DoubleArraySubscriber botPoseBlueMegaTag1Sub;
    private final DoubleArraySubscriber botPoseBlueMegaTag2Sub;
    private final IntegerSubscriber tidSub;
    private final IntegerSubscriber priorityIdSub;
    private final DoubleArrayPublisher robotOrientationPub;

    // Limelight info set via periodic()
    private TimestampedDoubleArray botPoseBlueMegaTag1;
    private TimestampedDoubleArray botPoseBlueMegaTag2;
    private double tx;
    private double ty;
    private double ta;
    private double tl;
    private long tid;
    private long priorityId;

    // For publishing the limelight pose
    private final NetworkTable poseTable;
    private final DoubleArrayPublisher limelightPub;

    private final double fieldExtentMetresX;
    private final double fieldExtentMetresY;
    private final double maxAmbiguity;
    private final double highQualityAmbiguity;
    private final double maxVisposDeltaDistanceMetres;
    private final SwerveTelemetry telemetry;
    private PoseEstimate poseEstimate;
    private VisionPositionInfo visPosInfo;

    /**
     * Create a new VisionAwareSwerveDrive
     *
     * @param coreSwerveConfig the core configuration for the swerve drive
     * @param visionConfig the vision configuration for the swerve drive
     */
    public VisionAwareSwerveDrive(CoreSwerveConfig coreSwerveConfig, VisionConfig visionConfig) {
        super(coreSwerveConfig);
        // initialize swerve variables
        this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
        this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
        this.maxAmbiguity = visionConfig.maxAmbiguity();
        this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
        this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();

        // initialize vision data subscribers
        table = NetworkTableInstance.getDefault().getTable("limelight-" + visionConfig.limelightName());
        txSub = table.getDoubleTopic("tx").subscribe(-1);
        tySub = table.getDoubleTopic("ty").subscribe(-1);
        taSub = table.getDoubleTopic("ta").subscribe(-1);
        tlSub = table.getDoubleTopic("tl").subscribe(-1);
        botPoseBlueMegaTag1Sub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        botPoseBlueMegaTag2Sub = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
        tidSub = table.getIntegerTopic("tid").subscribe(-1);
        priorityIdSub = table.getIntegerTopic("priorityid").subscribe(-1);
        robotOrientationPub = table.getDoubleArrayTopic("robot_orientation_set").publish();

        // Initialize the NetworkTable for publihsing the LimeLightPose
        poseTable = NetworkTableInstance.getDefault().getTable("LimelightPose");
        limelightPub = poseTable.getDoubleArrayTopic("Robot").publish();

        // Set the Pipeline & Camera Mode based on config
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        pipeline.setNumber(visionConfig.pipelineAprilTagDetect());
        NetworkTableEntry camMode = table.getEntry("camMode");
        camMode.setNumber(visionConfig.camModeVision());

        // bootup values
        poseEstimate = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0, null);
        visPosInfo = new VisionPositionInfo(new Pose2d(), 0, VecBuilder.fill(0, 0, 0), NONE, 0);
        this.telemetry = coreSwerveConfig.telemetry();
    }

    public void periodic() {
        super.periodic();
        updateLimelight();
        updateOdometryFromVision();
        updateTelemetry();
    }

    /**
     * Reset gyro to 0 and switch back to MegaTag1 for orientation setting.
     */
    @Override
    public void zeroGyro() {
        super.zeroGyro();
        state = State.INITALIZE;
    }

    /**
     * Pull the latest data from the Limelight NetworkTables, while updating robot orientation from odometry to Limelight.
     */
    private void updateLimelight() {
        // Set Limelight Orientation
        Pose2d currentPose = getPose();
        setLimelightOrientation(currentPose.getRotation().getDegrees());

        // Get all info from Limelight NT
        botPoseBlueMegaTag1 = botPoseBlueMegaTag1Sub.getAtomic();
        tx = txSub.get();
        ty = tySub.get();
        ta = taSub.get();
        tl = tlSub.get();
        tid = tidSub.get();
        priorityId = priorityIdSub.get();

        // Must be called after SetOrientation
        botPoseBlueMegaTag2 = botPoseBlueMegaTag2Sub.getAtomic();
    }

    /**
     * Update the odometry tracking of the robot using vision data obtained from Limelight, if available.
     */
    protected void updateOdometryFromVision() {
        final String NOT_AVAILABLE = "hugh-free";
        if (table.getEntry("pipeline").getString(NOT_AVAILABLE).equals(NOT_AVAILABLE)) {
            // Vision system not available
            return;
        }

        VisionPositionInfo visPosInfo;
        PoseConfidence confidence;

        try {
            Pose2d odometryPose = super.getPose();
            visPosInfo = this.getVisionPositionInfo(odometryPose);
            confidence = visPosInfo.confidence();
        } catch (InvalidVisionDataException e) {
            visPosInfo = null;
            confidence = NONE;
            if (!RobotBase.isSimulation()) {
                System.out.println("Failure reading data from vision subsystem: " + e);
            }
        }

        if (confidence != NONE) {
            super.addVisionMeasurement(visPosInfo.pose(), visPosInfo.timestampSeconds(), visPosInfo.deviation());
        }
    }

    /**
     * Update the telemetry data for the VisionAwareSwerveDrive.
     */
    protected void updateTelemetry() {
        // todo: do we still need this?
        publishToField(poseEstimate.pose);
        if (telemetry.enabled) {
            telemetry.visionPoseUpdate = visPosInfo.confidence() != PoseConfidence.NONE;
            telemetry.visionPoseConfidence = visPosInfo.confidence();
            telemetry.visionPriorityId = priorityId;
            telemetry.visionTid = tid;
            telemetry.visionTx = tx;
            telemetry.visionTy = ty;
            telemetry.visionTa = ta;
            telemetry.visionTl = tl;
            telemetry.visionPoseX = poseEstimate.pose.getX();
            telemetry.visionPoseY = poseEstimate.pose.getY();
            telemetry.visionPoseHeading = poseEstimate.pose.getRotation().getDegrees();
            telemetry.visionTargetAvgDist = poseEstimate.avgTagDist;
            telemetry.visionNumTags = poseEstimate.tagCount;
            // todo: is this data required?
            telemetry.visionAprilTagInfo = aprilTagInfoArrayToString(poseEstimate.rawFiducials);
            telemetry.visionPoseSwerveDiff = visPosInfo == null ? Double.MIN_VALUE : visPosInfo.odometryDistDelta();
        }
    }

    /**
     * Compute the position information using limelight data and the current odometry pose.
     *
     * @param currentOdometryPose the current odometry pose
     * @return VisionPositionInfo.
     * @throws InvalidVisionDataException if the vision data is invalid
     */
    private VisionPositionInfo getVisionPositionInfo(Pose2d currentOdometryPose) throws InvalidVisionDataException {
        if (state.equals(State.INITALIZE)) {
            this.poseEstimate = getBotPoseEstimate(botPoseBlueMegaTag1);
            this.visPosInfo = calcVisionPositionInfo(poseEstimate, currentOdometryPose, false);
            // When we get a high confidence pose from MegaTag1, switch to MegaTag2
            if (visPosInfo.confidence() == PoseConfidence.HIGH) {
                state = State.READY;
            }
            return visPosInfo;
        } else {
            this.poseEstimate = getBotPoseEstimate(botPoseBlueMegaTag2);
            this.visPosInfo = calcVisionPositionInfo(poseEstimate, currentOdometryPose, true);
            return visPosInfo;
        }
    }

    private String aprilTagInfoArrayToString(RawFiducial[] rawFiducials) {
        if (rawFiducials == null) {
            return "null";
        }
        StringBuilder sb = new StringBuilder();
        for (RawFiducial rawFiducial : rawFiducials) {
            sb
                .append("[id:")
                .append(rawFiducial.id)
                .append(",distToRobot:")
                .append(rawFiducial.distToRobot)
                .append(",ambiguity:")
                .append(rawFiducial.ambiguity)
                .append(",txnc:")
                .append(rawFiducial.txnc)
                .append(",tync:")
                .append(rawFiducial.tync)
                .append(",ta:")
                .append(rawFiducial.ta)
                .append("]");
        }
        return sb.toString();
    }

    private static Pose2d toPose2D(double[] inData) throws InvalidVisionDataException {
        if (inData.length < 6) {
            throw new InvalidVisionDataException("Bad LL 2D Pose Data!");
        }

        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        // Add 180deg to rotation because Hugh is on rear of bot
        Rotation2d r2d = Rotation2d.fromDegrees((inData[5] + 180) % 360);
        return new Pose2d(tran2d, r2d);
    }

    private static double extractBotPoseEntry(double[] inData, int position) throws InvalidVisionDataException {
        if (inData.length < position + 1) {
            throw new InvalidVisionDataException(
                "Cannot reference position: " + position + " in data array of length: " + inData.length
            );
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(TimestampedDoubleArray botpose) throws InvalidVisionDataException {
        var poseArray = botpose.value;
        var pose = toPose2D(poseArray);
        double latency = extractBotPoseEntry(poseArray, 6);
        int tagCount = (int) extractBotPoseEntry(poseArray, 7);
        double tagSpan = extractBotPoseEntry(poseArray, 8);
        double tagDist = extractBotPoseEntry(poseArray, 9);
        double tagArea = extractBotPoseEntry(poseArray, 10);
        // getlastchange() in microseconds, ll latency in milliseconds
        var timestampSeconds = (botpose.serverTime / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length == expectedTotalVals) {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, timestampSeconds, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    private void publishToField(Pose2d llPose) {
        // If you have a Field2D you can easily push it that way here.
        limelightPub.set(new double[] { llPose.getX(), llPose.getY(), llPose.getRotation().getDegrees() });
    }

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     * <p>
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     * <p>
     *
     * @return position info or null
     * @since 2024-02-10
     */
    private VisionPositionInfo calcVisionPositionInfo(
        PoseEstimate poseEstimate,
        Pose2d odometryPose,
        boolean isMegaTag2
    ) {
        double stdDevRatio = 1310;
        double stdDevRatio2 = 5 * stdDevRatio;
        PoseConfidence poseConfidence = PoseConfidence.NONE;
        double compareDistance = poseEstimate.pose.getTranslation().getDistance(odometryPose.getTranslation());

        // If pose is 0,0 or no tags in view, we don't actually have data - return null
        if (
            poseEstimate.pose.getX() > 0 &&
            poseEstimate.pose.getY() > 0 &&
            poseEstimate.rawFiducials.length >= 1 &&
            poseEstimate.pose.getX() < fieldExtentMetresX &&
            poseEstimate.pose.getY() < fieldExtentMetresY &&
            getGyroYawRate() <= 720
        ) {
            // MegaTag2 are always high confedence if they exist.
            if (isMegaTag2) {
                poseConfidence = PoseConfidence.HIGH;
                stdDevRatio = 0.6;
                stdDevRatio2 = 9999999;
            }
            // MegaTag1 needs some calculation based on tags and distance
            else {
                // Get the "best" tag - assuming the first one is the best - TBD TODO
                RawFiducial rawFiducial = poseEstimate.rawFiducials[0];

                if (rawFiducial.ambiguity < maxAmbiguity) {
                    // If the ambiguity is very low, use the data as is (or when disabled, to allow for
                    // bot repositioning
                    if (rawFiducial.ambiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
                        stdDevRatio = .01;
                        stdDevRatio2 = 5 * stdDevRatio;
                        poseConfidence = PoseConfidence.HIGH;
                    } else {
                        // We need to be careful with this data set. If the location is too far off,
                        // don't use it. Otherwise, scale confidence by distance.
                        if (compareDistance < maxVisposDeltaDistanceMetres) {
                            stdDevRatio = Math.pow(rawFiducial.distToRobot, 2) / 2;
                            stdDevRatio2 = 5 * stdDevRatio;
                            poseConfidence = PoseConfidence.MEDIUM;
                        }
                    }
                }
            }
        }

        Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, stdDevRatio2);
        return new VisionPositionInfo(
            poseEstimate.pose,
            poseEstimate.timestampSeconds,
            deviation,
            poseConfidence,
            compareDistance
        );
    }

    private void setLimelightOrientation(double yaw) {
        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = entries[2] = entries[3] = entries[4] = entries[5] = 0;

        robotOrientationPub.set(entries);
    }
}
