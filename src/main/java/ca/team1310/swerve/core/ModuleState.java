package ca.team1310.swerve.core;

import ca.team1310.swerve.utils.Coordinates;

/**
 * @author Tony Field
 * @since 2025-02-01 19:19
 */
public class ModuleState {

    private Coordinates location;

    private double drivePosition, driveVelocity, driveOutputPower, anglePosition, angleVelocity, absoluteEncoderAngle, desiredSpeed, desiredAngle =
        0;

    ModuleState() {}

    public Coordinates getLocation() {
        return location;
    }

    void setLocation(Coordinates location) {
        this.location = location;
    }

    public double getPosition() {
        return drivePosition;
    }

    void setPosition(double drivePosition) {
        this.drivePosition = drivePosition;
    }

    void setVelocity(double driveVelocity) {
        this.driveVelocity = driveVelocity;
    }

    public double getDriveOutputPower() {
        return driveOutputPower;
    }

    void setDriveOutputPower(double driveOutputPower) {
        this.driveOutputPower = driveOutputPower;
    }

    void setAngle(double anglePosition) {
        this.anglePosition = anglePosition;
    }

    public double getAngleVelocity() {
        return angleVelocity;
    }

    void setAngleVelocity(double angleVelocity) {
        this.angleVelocity = angleVelocity;
    }

    public double getAbsoluteEncoderAngle() {
        return absoluteEncoderAngle;
    }

    void setAbsoluteEncoderAngle(double absoluteEncoderAngle) {
        this.absoluteEncoderAngle = absoluteEncoderAngle;
    }

    /**
     * Get the angle of the module
     * @return the angle of the module in degrees from -180 to 180 (ccw positive)
     */
    public double getAngle() {
        return anglePosition;
    }

    /**
     * Get the speed of the module
     * @return the speed of the degrees in m/s
     */
    public double getSpeed() {
        return driveVelocity;
    }

    public double getDesiredSpeed() {
        return desiredSpeed;
    }

    void setDesiredSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }
}
