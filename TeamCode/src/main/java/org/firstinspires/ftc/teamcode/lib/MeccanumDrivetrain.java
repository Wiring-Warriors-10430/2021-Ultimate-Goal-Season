package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class MeccanumDrivetrain {
    private DcMotorEx rearLeftDrive;
    private DcMotorEx rearRightDrive;
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;

    public MeccanumDrivetrain(DcMotorEx rearLeftDrive, DcMotorEx rearRightDrive, DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive) {
          this.rearLeftDrive = rearLeftDrive;
          this.rearRightDrive = rearRightDrive;
          this.frontLeftDrive = frontLeftDrive;
          this.frontRightDrive = frontRightDrive;
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
}
