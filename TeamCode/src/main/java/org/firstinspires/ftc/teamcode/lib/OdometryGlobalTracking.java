package org.firstinspires.ftc.teamcode.lib;

public class OdometryGlobalTracking implements Runnable {
    // mm
    private static final double odometerWheelDistance = 380;

    private double leftOdometer = 0, rightOdometer = 0, centerOdometer = 0;
    private double robotX = 0, robotY = 0, robotTheta = 0;

    private Encoder left, right, center;

    public OdometryGlobalTracking(Encoder left, Encoder right, Encoder center) {
        this.left = left;
        this.right = right;
        this.center = center;
    }

    @Override
    public void run() {
        double newLeft = left.getDistance(), newRight = right.getDistance(), newCenter = center.getDistance();
        double deltaLeft, deltaRight, deltaM, deltaX, deltaY, deltaTheta;

        deltaLeft = newLeft - leftOdometer;
        deltaRight = newRight - rightOdometer;

        deltaTheta = (deltaLeft - deltaRight) / (odometerWheelDistance);

        

        leftOdometer = newLeft;
        rightOdometer = newRight;

        robotTheta = robotTheta + deltaTheta;
    }

    public double getRobotThetaRad() {
        return robotTheta;
    }
}
