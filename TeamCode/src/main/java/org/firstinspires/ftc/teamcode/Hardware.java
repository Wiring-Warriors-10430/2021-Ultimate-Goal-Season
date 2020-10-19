package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.Odometry;
import org.firstinspires.ftc.teamcode.lib.OdometryGlobalTracking;

import java.io.File;

public class Hardware {
    public DcMotorEx rearLeftDrive;
    public DcMotorEx rearRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public Encoder left;
    public Encoder right;
    public Encoder center;

    public Odometry odometry;

    private double robotEncoderWheelDistance = 386;
    private double horizontalEncoderTickPerDegreeOffset;

    public final static double odometerToMM = (1 / 8192d) * (38d * Math.PI);

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Hardware () {

    }

    public void init(HardwareMap ahwMap) {
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

        // Save reference to Hardware map
        hwMap = ahwMap;

        rearLeftDrive = hwMap.get(DcMotorEx.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rearRightDrive");
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");

        left = new Encoder(rearLeftDrive, 8192, 1, odometerToMM, false);
        right = new Encoder(rearRightDrive, 8192, 1, odometerToMM, true);
        center = new Encoder(frontLeftDrive, 8192, 1, odometerToMM, false);

        odometry = new Odometry(left, right, center, robotEncoderWheelDistance, horizontalEncoderTickPerDegreeOffset, 75);

        odometry.startTracking();

        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

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
