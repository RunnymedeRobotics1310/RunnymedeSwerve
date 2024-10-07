package ca.team1310.swervedrive.vision;

class RawFiducial {
    public int    id;
    public double txnc;
    public double tync;
    public double ta;
    public double distToCamera;
    public double distToRobot;
    public double ambiguity;

    public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot,
        double ambiguity) {
        this.id           = id;
        this.txnc         = txnc;
        this.tync         = tync;
        this.ta           = ta;
        this.distToCamera = distToCamera;
        this.distToRobot  = distToRobot;
        this.ambiguity    = ambiguity;
    }
}