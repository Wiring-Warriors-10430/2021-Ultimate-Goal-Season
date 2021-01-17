package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.Odometry;

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

    public DcMotorEx shooterLift;
    public DcMotorEx shooter;

    public CRServo intake;
    public CRServo feeder;
    public Servo indexer;
    public Servo pusher;
    public CRServo wobbleLeft;
    public CRServo wobbleRight;
    public Servo sounderArm;

    public DistanceSensor sounder;

    private double robotEncoderWheelDistance = 386;
    private double horizontalEncoderTickPerDegreeOffset;

    public final static double odometerToMM = (1 / 8192d) * (38d * Math.PI);

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public boolean verbose = true;

    public Hardware () {/**empty constructor*/}

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /**
         *    Init Drivetrain and DcMotors
         */

        // DcMotors
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rearRightDrive");
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");

        shooterLift = hwMap.get(DcMotorEx.class, "shooterLift");
        shooter = hwMap.get(DcMotorEx.class, "shooter");

        // Drivetrain
        left = new Encoder(rearLeftDrive, 8192, 1, odometerToMM, false);
        right = new Encoder(rearRightDrive, 8192, 1, odometerToMM, true);
        center = new Encoder(frontLeftDrive, 8192, 1, odometerToMM, false);

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

        odometry = new Odometry(left, right, center, robotEncoderWheelDistance, horizontalEncoderTickPerDegreeOffset, 75);

        odometry.startTracking();

        // DcMotor Settings
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLift.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        shooterLift.setTargetPosition(0);
        shooterLift.setPower(0);
        shooter.setVelocity(0);

        /**
         *    Init Servos
         */

        // CR Servos
        intake = hwMap.get(CRServo.class, "intake");
        feeder = hwMap.get(CRServo.class, "feeder");
        wobbleLeft = hwMap.get(CRServo.class, "intake");
        wobbleRight = hwMap.get(CRServo.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setPower(0);
        feeder.setPower(0);
        wobbleLeft.setPower(0);
        wobbleRight.setPower(0);

        // Servos
        indexer = hwMap.get(Servo.class, "indexer");
        pusher = hwMap.get(Servo.class, "pusher");
        sounderArm = hwMap.get(Servo.class, "sounderArm");

        indexer.setDirection(Servo.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);
        sounderArm.setDirection(Servo.Direction.FORWARD);

        indexer.setPosition(.5);
        pusher.setPosition(.5);
        sounderArm.setPosition(.5);

        /**
         *    Init Sensors
         */

        // I2C Sensors
        sounder = hwMap.get(DistanceSensor.class, "sounder");
    }
}
