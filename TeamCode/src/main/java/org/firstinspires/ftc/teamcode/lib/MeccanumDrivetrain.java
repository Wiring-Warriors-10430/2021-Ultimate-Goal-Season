package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class MeccanumDrivetrain {
    private DcMotorEx rearLeftDrive;
    private DcMotorEx rearRightDrive;
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;

    private Odometry odometry;

    private PIDFController xPID = new PIDFController(.012,0,0,0, 10, 50, .2); //kD = .1
    private PIDFController yPID = new PIDFController(.007,0,.3,0, 10, 50, .2); // kD = .1
    private PIDFController thetaPID = new PIDFController(3.5,0,30,0, Math.toRadians(.75), Math.toRadians(.5), 0); // kP = 3.9

    private double goalX = 0;
    private double goalY = 0;
    private double goalHeading = 0;

    private double maxVoltage = 13;

    private double currentVoltage = 12.5;

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

        xOutput = MoreMath.clamp(xOutput, -1, 1);
        yOutput = MoreMath.clamp(yOutput, -1, 1);
        headingOutput = MoreMath.clamp(headingOutput, -1, 1);

        xOutput = scalePower(xOutput);
        yOutput = scalePower(yOutput);
        headingOutput = scalePower(headingOutput);

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

    public void setVoltage(double voltage) {
        currentVoltage = voltage;
    }

    public double getVoltage() {
        return currentVoltage;
    }

    private double scalePower(double desiredPower) {
        return desiredPower * (maxVoltage/currentVoltage);
    }

    public boolean thetaAt() {
        return thetaPID.isRunning();
    }

    public boolean xAt() {
        return xPID.isRunning();
    }

    public boolean yAt() {
        return yPID.isRunning();
    }
}
