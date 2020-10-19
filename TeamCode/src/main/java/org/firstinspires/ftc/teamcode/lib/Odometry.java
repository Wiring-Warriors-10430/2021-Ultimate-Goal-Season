package org.firstinspires.ftc.teamcode.lib;

public class Odometry {
    private OdometryGlobalTracking tracking;

    private Encoder left;
    private Encoder right;
    private Encoder normal;

    private double trackWidth;
    private double normalOffset;

    private Thread positionThread;

    public Odometry(Encoder left, Encoder right, Encoder normal, double trackWidth, double normalOffset, int updateTime) {
        this.left = left;
        this.right = right;
        this.normal = normal;
        this.trackWidth = trackWidth;
        this.normalOffset = normalOffset;

        tracking = new OdometryGlobalTracking(left, right, normal, normalOffset, trackWidth, updateTime);

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        positionThread = new Thread(tracking);
    }

    public Pose2D getPos2D() {
        return new Pose2D(tracking.getRobotX(), tracking.getRobotY(), tracking.getRobotHeading());
    }

    public double getX() {
        return tracking.getRobotX();
    }

    public double getY() {
        return tracking.getRobotY();
    }

    public double getHeadingTheta() {
        return tracking.getRobotHeading();
    }

    public double getHeadingDeg() {
        return Math.toDegrees(getHeadingTheta());
    }

    public void startTracking() {
        positionThread.start();
    }

    public void stopTracking() {
        tracking.stop();
    }
}
