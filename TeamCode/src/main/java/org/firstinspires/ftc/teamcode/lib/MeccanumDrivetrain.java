package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class MeccanumDrivetrain {
    private DcMotorEx rearLeftDrive;
    private DcMotorEx rearRightDrive;
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;

    private Odometry odometry;

    private PIDFController xPID = new PIDFController(.011,0,.1,0, 5, 100);
    private PIDFController yPID = new PIDFController(.011,0,.1,0, 5, 100);
    private PIDFController thetaPID = new PIDFController(5.5,0,20,0, Math.toRadians(1), 100);

    private double goalX = 0;
    private double goalY = 0;
    private double goalHeading = 0;

    private double maxVoltage = 12.5;

    public MeccanumDrivetrain(DcMotorEx rearLeftDrive, DcMotorEx rearRightDrive, DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive, Odometry odometry) {
          this.rearLeftDrive = rearLeftDrive;
          this.rearRightDrive = rearRightDrive;
          this.frontLeftDrive = frontLeftDrive;
          this.frontRightDrive = frontRightDrive;

          this.odometry = odometry;
    }

    public void driveFieldCentered(double x, double y, double theta, double heading) {
        double adjustedY = y * Math.cos(heading) - x * Math.sin(heading);
        double adjustedX = y * Math.sin(heading) + x * Math.cos(heading);

        drive (adjustedX, adjustedY, theta);
    }

    public void drive(double x, double y, double theta) {
        double frontLeftPow = x + y + theta;
        double rearLeftPow = -x + y + theta;
        double frontRightPow = -x + y - theta;
        double rearRightPow = x + y - theta;

        /** DONT USE MAKES SLOW
        // Q1
        if (x >= 0 && y >= 0) {
            frontLeftPow = Range.scale(frontLeftPow, 0, 3, 0, 1);
            rearRightPow = Range.scale(rearRightPow, 0, 2, 0, 1);
        }

        // Q2
        if (x <= 0 && y >= 0) {
            rearLeftPow = Range.scale(rearLeftPow, 0, 2, 0, 1);
            frontRightPow = Range.scale(frontRightPow, 0, 2, 0, 1);
        }

        // Q3
        if (x <= 0 && y <= 0) {
            frontLeftPow = Range.scale(frontLeftPow, 0, -3, 0, -1);
            rearRightPow = Range.scale(rearRightPow, 0, -2, 0, -1);
        }

        // Q4
        if (x >= 0 && y <= 0) {
            rearLeftPow = Range.scale(rearLeftPow, 0, -2, 0, -1);
            frontRightPow = Range.scale(frontRightPow, 0, -2, 0, -1);
        } */

        rearLeftDrive.setPower(rearLeftPow);
        rearRightDrive.setPower(rearRightPow);
        frontLeftDrive.setPower(frontLeftPow);
        frontRightDrive.setPower(frontRightPow);
    }

    public void updateAutoDrive() {
        double xOutput = xPID.run(odometry.getX());
        double yOutput = yPID.run(odometry.getY());
        double headingOutput = -thetaPID.run(odometry.getHeadingTheta());

        driveFieldCentered(xOutput, yOutput, headingOutput, odometry.getHeadingTheta());
    }

    public void setGoal(double newX, double newY, double newHeading) {
        xPID.setTarget(newX);
        yPID.setTarget(newY);
        thetaPID.setTarget(newHeading);

        goalX = newX;
        goalY = newY;
        goalHeading = newHeading;
    }

    public boolean isRunning() {
        return xPID.isRunning() || yPID.isRunning() || thetaPID.isRunning();
    }
}
