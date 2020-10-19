package org.firstinspires.ftc.teamcode.lib;

public class OdometryGlobalTracking implements Runnable {
    // mm
    private double odometerTrackWidth = 0;
    // mm/rad
    private double normalOffset;

    private double leftOdometer = 0, rightOdometer = 0, centerOdometer = 0;
    private double robotX = 0, robotY = 0, robotTheta = 0;

    private Encoder left, right, center;

    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread - threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
    private int sleepTime;

    public OdometryGlobalTracking(Encoder left, Encoder right, Encoder center, double normalOffset, double odometerTrackWidth, int updateTime) {
        this.left = left;
        this.right = right;
        this.center = center;
        this.normalOffset = normalOffset;
        this.odometerTrackWidth = odometerTrackWidth;

        sleepTime = updateTime;
    }

    public void positionUpdate() {
        double newLeft = left.getDistance(), newRight = right.getDistance(), newCenter = center.getDistance();

        double deltaLeft = newLeft - leftOdometer;
        double deltaRight = newRight - rightOdometer;
        double deltaCenter = newCenter - centerOdometer;

        double deltaTheta = ( (deltaLeft - deltaRight) / (odometerTrackWidth) );
        robotTheta += (deltaTheta);

        double localDeltaX = deltaCenter - (deltaTheta * normalOffset);

        double averageDeltaY = (deltaLeft + deltaRight) / 2;

        robotX = robotX + (averageDeltaY*Math.sin(robotTheta) + localDeltaX*Math.cos(robotTheta));
        robotY = robotY + (averageDeltaY*Math.cos(robotTheta) - localDeltaX*Math.sin(robotTheta));

        leftOdometer = newLeft;
        rightOdometer = newRight;
        centerOdometer = newCenter;
    }

    public double getRobotHeading() {
        // Negate to work on the unit Circle.
        return -robotTheta;
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            positionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Stops the position update thread
     */
    public void stop(){
        isRunning = false;
    }
}
