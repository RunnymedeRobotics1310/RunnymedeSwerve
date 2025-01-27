package ca.team1310.swerve.odometry;

import ca.team1310.swerve.SwerveTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.function.DoubleConsumer;

/**
 * Represents a gyro that can be used to determine the orientation of the robot.
 */
public interface Gyro extends Sendable {
    /**
     * Called at the start of each robot period. Used to read sensor values
     */
    void periodic();

    /**
     * Reset pitch, yaw, and roll to 0 degrees.
     */
    void zeroGyro();

    /**
     * Get the roll of the robot, in degrees.
     * @return the roll of the robot, in degrees
     */
    double getRoll();

    /**
     * Get the pitch of the robot, in degrees.
     * @return the pitch of the robot, in degrees
     */
    double getPitch();

    /**
     * Get the yaw of the robot, in degrees.
     * @return the yaw of the robot, in degrees
     */
    double getYaw();

    /**
     * Get the rate of yaw change of the robot, in degrees.
     * @return the rate of rotation of the yaw of the robot, in degrees per second
     */
    double getYawRate();

    /**
     * Update the gyro in simulation mode. Not used in normal operation
     *
     * @param kinematics The kinematics of the swerve drive
     * @param states The states of the swerve modules
     * @param modulePoses The poses of the swerve modules
     * @param field The field object
     */
    void updateOdometryForSimulation(
        SwerveDriveKinematics kinematics,
        SwerveModuleState[] states,
        Pose2d[] modulePoses,
        Field2d field
    );

    /**
     * Populate the telemetry object with the gyro's data.
     *
     * @param telemetry The telemetry object to populate
     */
    void populateTelemetry(SwerveTelemetry telemetry);

    /**
     * Get the rotation of the robot as a Rotation2d object.
     * @return the rotation of the robot as a Rotation2d object
     */
    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    default void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getYaw, (DoubleConsumer) null);
    }
}
