package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.MoreMath;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

import java.util.List;

@TeleOp(name="Teleop2: Electric Boogaloo", group="aaRobot2")
public class TeleopElectricBoogaloo extends OpMode {
    HardwareElectricBoogaloo robot = new HardwareElectricBoogaloo();

    Joystick leftStick;
    Joystick rightStick;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.odometry.setOffsetFromFile();

        robot.odometry.setOffset(0, 0, Math.toRadians(0));

        leftStick = new Joystick(gamepad1, Joystick.Stick.LEFT, 3);
        rightStick = new Joystick(gamepad1, Joystick.Stick.RIGHT, 3);

        robot.drivetrain.setGoal(1828, 3048, Math.toRadians(Math.toRadians(15)));

        robot.tfod.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        robot.tfod.setZoom(4, 16.0/9.0);

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        timer.startTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
        } else {
            telemetry.log().clear();
            telemetry.log().add("Gyro Calibrated. Press Start.");
            telemetry.clear();
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.log().clear();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            robot.intake.setPower(0.3);
        } else if (gamepad1.right_bumper) {
            robot.intake.setPower(-0.3);
        } else {
            robot.intake.setPower(-.1);
        }

        if (robot.verbose) {
            visionTest();
            verboseOutput();
        }

        if (gamepad1.a) {
            robot.drivetrain.updateAutoDrive();
        } else {
            robot.drivetrain.drive(leftStick.getX(1), leftStick.getY(1),
                    rightStick.getX(1));
        }

        //robot.odometry.writePoseToFile();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.odometry.setOffset(0, 0, 0);
    }

    private void goodWait(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < time) {}
    }

    private void visionTest() {
        if (robot.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                //telemetry.update();
            }
        }
    }

    private void verboseOutput() {
        // Important Stuff rn.

        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();

        // Send Odometry info
        telemetry.addLine("Odometry:");
        telemetry.addData("Est. Theta", MoreMath.round(robot.odometry.getHeadingTheta(), 2));
        telemetry.addData("X", MoreMath.round(robot.odometry.getX(), 2));
        telemetry.addData("Y", MoreMath.round(robot.odometry.getY(), 2));
        telemetry.addData("Left", robot.left.getDistance());
        telemetry.addData("Right", robot.right.getDistance());
        telemetry.addData("Center", robot.center.getDistance());
        telemetry.addData("Conversion", robot.odometerToMM);
        telemetry.addLine("");
        telemetry.addLine();

        // Send Joystick Info
        telemetry.addLine("Sticks");
        telemetry.addLine("leftStick:");
        telemetry.addData("    X", leftStick.getX(1));
        telemetry.addData("    Y", leftStick.getY(1));
        telemetry.addLine("rightStick:");
        telemetry.addData("    X", rightStick.getX(1));
        telemetry.addData("    Y", rightStick.getY(1));
        telemetry.addLine();
        telemetry.addLine();

        // Send Motor info
        telemetry.addLine("Motors:");
        telemetry.addData("rearLeftDrive power", robot.rearLeftDrive.getPower());
        telemetry.addData("rearLeftDrive port", robot.rearLeftDrive.getPortNumber());
        telemetry.addData("rearRightDrive power", robot.rearRightDrive.getPower());
        telemetry.addData("rearRightDrive port", robot.rearRightDrive.getPortNumber());
        telemetry.addData("frontLeftDrive power", robot.frontLeftDrive.getPower());
        telemetry.addData("frontLeftDrive port", robot.frontLeftDrive.getPortNumber());
        telemetry.addData("frontRightDrive power", robot.frontRightDrive.getPower());
        telemetry.addData("frontRightDrive port", robot.frontRightDrive.getPortNumber());
        telemetry.addLine();
        telemetry.addData("shooterLift power", robot.intake.getPower());
        telemetry.addData("shooterLift port", robot.intake.getPortNumber());
        telemetry.addLine();
        telemetry.addLine();

        // Send Servo info
        telemetry.addLine("Servos:");
        telemetry.addLine();
        telemetry.addLine();

        // Send Sensor info
        telemetry.addLine("Sensors:");
        telemetry.addLine("NavX Micro:");
        AngularVelocity rates = robot.navxMicro.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = robot.navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("    dx", robot.formatRate(rates.xRotationRate))
                .addData("    dy", robot.formatRate(rates.yRotationRate))
                .addData("    dz", "%s deg/s", robot.formatRate(rates.zRotationRate));
        telemetry.addLine()
                .addData("    heading", robot.formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("    roll", robot.formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("    pitch", "%s deg", robot.formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addLine();
        telemetry.addLine();
    }
}
