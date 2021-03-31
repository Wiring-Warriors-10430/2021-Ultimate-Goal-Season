package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.MeccanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.Odometry;
import org.firstinspires.ftc.teamcode.lib.PIDFController;

import java.io.File;
import java.util.List;

public class Hardware {
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

    public PIDFController shooterVelocityController;

    public CRServo intake;
    public CRServo feeder;
    public Servo indexer;
    public Servo pusher;
    public Servo intakeFloor;
    public CRServo wobbleLeft;
    public CRServo wobbleRight;
    public Servo sounderArm;

    //public DistanceSensor highSounder;
    //public DistanceSensor lowSounder;

    public NavxMicroNavigationSensor navxMicro;

    public VoltageSensor voltageSensor;

    private double robotEncoderWheelDistance = 386;
    private double horizontalEncoderTickPerDegreeOffset;

    public final static double odometerToMM = (1 / 8192d) * (38d * Math.PI);

    public final static double shooterLiftToDeg = (360d/(8192d));
    public final static double wobbleLiftToMM = (1/383.6d) * (47.625d * Math.PI);
    public final static double wobbleArmReduction = 4.5;
    public final static double wobbleArmToDeg = 360d / ((753.2d) * wobbleArmReduction);

    private double tol = 50;   // Set to the amount of ticks per second you are ok being off.
    private double shooterRPM = 3500;   // TODO: Set to desired RPM of motor. //5185
    private double shooterRPS = shooterRPM / 60;
    public double desiredSpeed = shooterRPS*28; // shooterRPS * 28

    private double autoTol = 50;   // Set to the amount of ticks per second you are ok being off.
    private double autoShooterRPM = 3050; //3000   // TODO: Set to desired RPM of motor. //5185
    private double autoShooterRPS = autoShooterRPM / 60;
    public double autoDesiredSpeed = autoShooterRPS*28; // shooterRPS * 28

    public static final double shooterAngle = 28.5;
    public static final double autoShooterAngle = 28.5;

    public static final double indexerLowest = .8; //.8
    public static final double indexerLow = .38; //.39
    public static final double indexerMid = .29; //.31
    public static final double indexerHigh = .2; //.23

    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private File vuforiaKey = AppUtil.getInstance().getSettingsFile("vuforiaKey.txt");

    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static String VUFORIA_KEY;

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public boolean verbose = true;

    public Hardware () {/**empty constructor*/}

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        VUFORIA_KEY = ReadWriteFile.readFile(vuforiaKey).trim();


        /**
         *    Init Drivetrain and DcMotors
         */

        // Voltage Sensor
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");

        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");

        // DcMotors
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rearLeftDrive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rearRightDrive");
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");

        shooterLift = hwMap.get(DcMotorEx.class, "shooterLift");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        wobbleLift = hwMap.get(DcMotorEx.class, "wobbleLift");
        wobbleArm = hwMap.get(DcMotorEx.class, "wobbleArm");

        shooterLiftEnc = new Encoder(shooterLift, 8192, 1, shooterLiftToDeg,true);
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

        odometry = new Odometry(left, right, center, (IntegratingGyroscope)navxMicro, robotEncoderWheelDistance, horizontalEncoderTickPerDegreeOffset, 50);

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        wobbleArm.setPower(0);
        wobbleLift.setPower(0);

        shooterVelocityController = new PIDFController(.01, 0, 0, .3, 10, 0, .3, true);

        shooterLiftController = new PIDFController(.04, 0, 0, 0, .2, 0, 0);
        wobbleLiftController = new PIDFController(.03, 0, 0, 0, 1, 0, 0);
        wobbleArmController = new PIDFController(.01, 0, 0, 0, 1, 0, 0);

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
        intakeFloor = hwMap.get(Servo.class, "ramp servo");

        indexer.setDirection(Servo.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);
        sounderArm.setDirection(Servo.Direction.FORWARD);
        intakeFloor.setDirection(Servo.Direction.FORWARD);

        // .2 - .8 because linear servo, .2 - .6 because mechanics
        indexer.setPosition(.8);
        pusher.setPosition(.22);
        // .2 - .8 because linear servo
        sounderArm.setPosition(.2);
        intakeFloor.setPosition(.52);


        /**
         *    Init Sensors
         */

        // I2C Sensors
        //lowSounder = hwMap.get(DistanceSensor.class, "low_sounder");
        //highSounder = hwMap.get(DistanceSensor.class, "high_sounder");

        // Vision System
        initVuforia();
        initTfod();
    }

    public boolean shooterAtSpeed() {
        if ((shooter.getVelocity() > desiredSpeed - tol) && (shooter.getVelocity() < desiredSpeed + tol)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean autoShooterAtSpeed() {
        if ((shooter.getVelocity() > autoDesiredSpeed - tol) && (shooter.getVelocity() < autoDesiredSpeed + tol)) {
            return true;
        } else {
            return false;
        }
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(4, 16.0/9.0);
    }

    public double getHeading() {
        Orientation angles = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    public Depot measureStack() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions != null && recognitions.size() > 0) {
                Recognition recognition = recognitions.get(0);

                if (recognition.getLabel() == "Single") {
                    return Depot.MIDDLE;
                } else {
                    return Depot.BACK;
                }
            } else {
                return Depot.FRONT;
            }
        } else {
            return Depot.FRONT;
        }
    }

    public enum Depot {
        FRONT, MIDDLE, BACK
    }
}
