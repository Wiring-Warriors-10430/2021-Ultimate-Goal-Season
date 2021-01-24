package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Odometry {
    private OdometryGlobalTracking tracking;

    private Encoder left;
    private Encoder right;
    private Encoder normal;

    private double trackWidth;
    private double normalOffset;

    private Thread positionThread;

    private double xOffset;
    private double yOffset;
    private double headingOffset;

    private File offsetX = AppUtil.getInstance().getSettingsFile("offsetX.txt");
    private File offsetY = AppUtil.getInstance().getSettingsFile("offsetY.txt");
    private File offsetTheta = AppUtil.getInstance().getSettingsFile("offsetTheta.txt");

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

    public void setOffset(Pose2D initial) {
        setOffset(initial.x, initial.y, initial.theta);
    }

    public void setOffset(double xOffset, double yOffset, double headingOffsetRad) {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.headingOffset = headingOffsetRad;
    }

    public Pose2D getPos2D() {
        return new Pose2D(tracking.getRobotX(), tracking.getRobotY(), tracking.getRobotHeading());
    }

    public double getX() {
        return tracking.getRobotX() + xOffset;
    }

    public double getY() {
        return tracking.getRobotY() + yOffset;
    }

    public double getHeadingTheta() {
        return tracking.getRobotHeading();
    }

    public double getHeadingDeg() {
        return Math.toDegrees(getHeadingTheta()) + Math.toDegrees(headingOffset);
    }

    public void writePoseToFile() {
        if (offsetX.canWrite()) {
            ReadWriteFile.writeFile(offsetX, String.valueOf(getX()));
        }
        if (offsetY.canWrite()) {
            ReadWriteFile.writeFile(offsetY, String.valueOf(getY()));
        }
        if (offsetTheta.canWrite()) {
            ReadWriteFile.writeFile(offsetTheta, String.valueOf(getHeadingTheta()));
        }
    }

    public Pose2D readPoseFromFile() {
        return new Pose2D(readXFromFile(), readYFromFile(), readHeadingFromFile());
    }

    public double readXFromFile() {
        return Double.parseDouble(ReadWriteFile.readFile(offsetX).trim());
    }

    public double readYFromFile() {
        return Double.parseDouble(ReadWriteFile.readFile(offsetY).trim());
    }

    public double readHeadingFromFile() {
        return Double.parseDouble(ReadWriteFile.readFile(offsetTheta).trim());
    }

    public void setOffsetFromFile() {
        setOffset(readPoseFromFile());
    }

    public void startTracking() {
        positionThread.start();
    }

    public void stopTracking() {
        tracking.stop();
    }
}
