package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class MeccanumDrivetrain {
    private DcMotorEx rearLeftDrive;
    private DcMotorEx rearRightDrive;
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;

    private Odometry odometry;

    private PIDFController xPID = new PIDFController(1,0,0,0);
    private PIDFController yPID = new PIDFController(1,0,0,0);
    private PIDFController thetaPID = new PIDFController(1,0,0,0);

    private int goalX = 0;
    private int goalY = 0;
    private int goalHeading = 0;

    public MeccanumDrivetrain(DcMotorEx rearLeftDrive, DcMotorEx rearRightDrive, DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive, Odometry odometry) {
          this.rearLeftDrive = rearLeftDrive;
          this.rearRightDrive = rearRightDrive;
          this.frontLeftDrive = frontLeftDrive;
          this.frontRightDrive = frontRightDrive;

          this.odometry = odometry;
    }

    public void drive(double x, double y, double theta) {
        double frontLeftPow = x + y + theta;
        double rearLeftPow = -x + y + theta;
        double frontRightPow = -x + y - theta;
        double rearRightPow = x + y - theta;

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
            frontLeftPow = Range.scale(frontLeftPow, 0, -3, 0, 1);
            rearRightPow = Range.scale(rearRightPow, 0, -2, 0, 1);
        }

        // Q4
        if (x >= 0 && y <= 0) {
            rearLeftPow = Range.scale(rearLeftPow, 0, -2, 0, 1);
            frontRightPow = Range.scale(frontRightPow, 0, -2, 0, 1);
        }

        rearLeftDrive.setPower(rearLeftPow);
        rearRightDrive.setPower(rearRightPow);
        frontLeftDrive.setPower(frontLeftPow);
        frontRightDrive.setPower(frontRightPow);
    }

    public void updateAutoDrive() {
        double xOutput = xPID.run(odometry.getX());
        double yOutput = yPID.run(odometry.getY());
        double headingOutput = thetaPID.run(odometry.getHeadingTheta());

        drive(xOutput, yOutput, headingOutput);
    }

    public void setGoal(int newX, int newY, int newHeading) {
        xPID.setTarget(newX);
        yPID.setTarget(newY);
        thetaPID.setTarget(newHeading);

        goalX = newX;
        goalY = newY;
        goalHeading = newHeading;
    }
}
