package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.MeccanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.Odometry;
import org.firstinspires.ftc.teamcode.lib.PIDFController;

import java.io.File;

public class Hardware {
    public static final double shooterAngle = 26;

    public DcMotorEx rearLeftDrive;
    public DcMotorEx rearRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public Encoder left;
    public Encoder right;
    public Encoder center;

    public Odometry odometry;

    public MeccanumDrivetrain drivetrain;

    public DcMotorEx shooterLift;
    public DcMotorEx shooter;
    public DcMotorEx wobbleLift;
    public DcMotorEx wobbleArm;

    public Encoder shooterLiftEnc;
    public Encoder wobbleLiftEnc;
    public Encoder wobbleArmEnc;

    public PIDFController shooterLiftController;
    public PIDFController wobbleLiftController;
    public PIDFController wobbleArmController;

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

    public final static double shooterLiftToDeg = (360d/(2786d*2));
    public final static double wobbleLiftToMM = (1/383.6d) * (47.625d * Math.PI);
    public final static double wobbleArmReduction = 4.5;
    public final static double wobbleArmToDeg = 360d / ((753.2d) * wobbleArmReduction);

    private double tol = 50;   // Set to the amount of ticks per second you are ok being off.
    private double shooterRPM = 5185;   // TODO: Set to desired RPM of motor. //5035
    private double shooterRPS = shooterRPM / 60;
    public double desiredSpeed = shooterRPS*28; // shooterRPS * 28

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
        wobbleLift = hwMap.get(DcMotorEx.class, "wobbleLift");
        wobbleArm = hwMap.get(DcMotorEx.class, "wobbleArm");

        shooterLiftEnc = new Encoder(shooterLift, 8192, 1, shooterLiftToDeg,false);
        wobbleArmEnc = new Encoder(wobbleArm, 8192, 1, wobbleArmToDeg,false);
        wobbleLiftEnc = new Encoder(wobbleLift, 8192, 1, wobbleLiftToMM,false);

        // Drivetrain
        left = new Encoder(rearLeftDrive, 8192, 1, odometerToMM, true);
        right = new Encoder(frontLeftDrive, 8192, 1, odometerToMM, false);
        center = new Encoder(rearRightDrive, 8192, 1, odometerToMM, true);

        left.reset();
        right.reset();
        center.reset();

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
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLift.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleLift.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleArm.setDirection(DcMotorSimple.Direction.REVERSE);

        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wobbleLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        shooterLift.setPower(0);
        shooter.setVelocity(0);

        wobbleArm.setPower(0);
        wobbleLift.setPower(0);

        shooter.setVelocityPIDFCoefficients(2, 0, 0, 0);

        shooterLiftController = new PIDFController(.15, 0, 0, 0, .2, 0);
        wobbleLiftController = new PIDFController(.03, 0, 0, 0, 1, 0);
        wobbleArmController = new PIDFController(.01, 0, 0, 0, 1, 0);

        // Drivetrain class
        drivetrain = new MeccanumDrivetrain(rearLeftDrive, rearRightDrive,frontLeftDrive, frontRightDrive, odometry);

        /**
         *    Init Servos
         */

        // CR Servos
        intake = hwMap.get(CRServo.class, "intake");
        feeder = hwMap.get(CRServo.class, "feeder");
        wobbleLeft = hwMap.get(CRServo.class, "wobbleLeft");
        wobbleRight = hwMap.get(CRServo.class, "wobbleRight");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // .2 - .8 because linear servo, .2 - .6 because mechanics
        indexer.setPosition(.8);
        pusher.setPosition(.22);
        // .2 - .8 because linear servo
        sounderArm.setPosition(.2);

        /**
         *    Init Sensors
         */

        // I2C Sensors
        sounder = hwMap.get(DistanceSensor.class, "sounder");
    }

    public boolean shooterAtSpeed() {
        if (shooter.getVelocity() > desiredSpeed - tol) {
            return true;
        } else {
            return false;
        }
    }
}
