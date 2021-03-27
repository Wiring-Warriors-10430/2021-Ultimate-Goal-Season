package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.MeccanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.Odometry;
import org.firstinspires.ftc.teamcode.lib.PIDFController;

import java.io.File;

public class HardwareElectricBoogaloo {
    public DcMotorEx rearLeftDrive;
    public DcMotorEx rearRightDrive;
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;

    public Encoder left;
    public Encoder right;
    public Encoder center;

    public Odometry odometry;

    public MeccanumDrivetrain drivetrain;

    public DcMotorEx intake;

    public NavxMicroNavigationSensor navxMicro;

    public VoltageSensor voltageSensor;

    private double robotEncoderWheelDistance = 386;
    private double horizontalEncoderTickPerDegreeOffset;

    public final static double odometerToMM = (1 / 8192d) * (38d * Math.PI);

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public boolean verbose = true;


    public HardwareElectricBoogaloo () {/**empty constructor*/}


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

        intake = hwMap.get(DcMotorEx.class, "intake");


        // Drivetrain
        left = new Encoder(rearLeftDrive, 8192, 1, odometerToMM, true);
        right = new Encoder(frontLeftDrive, 8192, 1, odometerToMM, false);
        center = new Encoder(rearRightDrive, 8192, 1, odometerToMM, true);

        left.reset();
        right.reset();
        center.reset();

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

        odometry = new Odometry(left, right, center, robotEncoderWheelDistance, horizontalEncoderTickPerDegreeOffset, 50);

        odometry.startTracking();

        drivetrain = new MeccanumDrivetrain(rearLeftDrive, rearRightDrive,frontLeftDrive, frontRightDrive, odometry);


        // Drivetrain Settings
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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


        // DcMotor Settings

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);


        /**
         *    Init Servos
         */

        // CR Servos


        // Servos


        /**
         *    Init Sensors
         */

        // Voltage Sensor
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");


        // I2C Sensors
        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");


    }


    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
