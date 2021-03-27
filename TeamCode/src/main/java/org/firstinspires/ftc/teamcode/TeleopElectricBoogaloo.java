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
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.MoreMath;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

@TeleOp(name="Teleop2: Electric Boogaloo", group="__Robot2")
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

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (robot.navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            goodWait(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
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
