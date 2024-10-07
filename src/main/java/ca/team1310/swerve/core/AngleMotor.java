package ca.team1310.swervedrive.core;

public interface AngleMotor {

    /**
     * Get the current position of the angle motor in degrees
     */
    double getPosition();

    /**
     * Set the target angle of the motor
     * 
     * @param degrees
     */
    void setReferenceAngle(double degrees);


    /**
     * Reset the internal encoder of this motor to the current position
     */
    void setEncoderPosition(double actualAngleDegrees);
}
