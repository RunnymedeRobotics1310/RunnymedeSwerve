package ca.team1310.swerve.odometry;

/**
 * Get the pose of an object on the field. Units are Metres and Degrees
 * @author Tony Field
 * @since 2025-02-01 23:29
 */
public class FieldPose {

    double x = 0;
    double y = 0;
    double theta = 0;

    public FieldPose() {}

    public FieldPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    /**
     * Get the angle of the robot in degrees
     * @return
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Set the angle of the robot in degrees
     * @param theta
     */
    public void setTheta(double theta) {
        this.theta = theta;
    }
}
