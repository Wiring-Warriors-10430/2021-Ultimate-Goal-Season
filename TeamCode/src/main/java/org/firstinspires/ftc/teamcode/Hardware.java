package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.OdometryGlobalTracking;

public class Hardware {
    public DcMotorEx rearLeftDrive;
    public DcMotorEx rearRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public Encoder left;
    public Encoder right;
    public Encoder center;

    public final static double odometerToMM = (1 / 8192d) * (37.3 * Math.PI) ;

    public OdometryGlobalTracking odometry;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Hardware () {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        rearLeftDrive = hwMap.get(DcMotorEx.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rearRightDrive");
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");

        left = new Encoder(rearLeftDrive, 8192, 1, odometerToMM);
        right = new Encoder(rearRightDrive, 8192, 1, odometerToMM);
        center = new Encoder(frontLeftDrive, 8192, 1, odometerToMM);

        odometry = new OdometryGlobalTracking(left, right, center);

        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
    }
}
