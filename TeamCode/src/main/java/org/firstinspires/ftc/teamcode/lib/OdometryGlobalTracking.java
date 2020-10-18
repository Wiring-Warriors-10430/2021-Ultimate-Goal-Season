package org.firstinspires.ftc.teamcode.lib;

public class OdometryGlobalTracking implements Runnable {
    // mm
    private static final double odometerWheelDistance = 386;

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

        double deltaLeft = newLeft - leftOdometer;
        double deltaRight = newRight - rightOdometer;
        double deltaCenter = newCenter - centerOdometer;

        double deltaTheta = ( (deltaLeft - deltaRight) / (odometerWheelDistance) );
        robotTheta += -(deltaTheta);

        /** TODO: Multiply deltaTheta by rad to mm conversion factor. In order to get that value,
         * write a calibration class */
        double localDeltaX = deltaCenter - deltaTheta;

        double averageDeltaY = (deltaLeft + deltaRight) / 2;

        robotX = robotX + (averageDeltaY*Math.sin(robotTheta) + localDeltaX*Math.cos(robotTheta));
        robotY = robotY + (averageDeltaY*Math.cos(robotTheta) - localDeltaX*Math.sin(robotTheta));

        leftOdometer = newLeft;
        rightOdometer = newRight;
        centerOdometer = newCenter;
    }

    public double getRobotThetaRad() {
        return robotTheta;
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }
}
