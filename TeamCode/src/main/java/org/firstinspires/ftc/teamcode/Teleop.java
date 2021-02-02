package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.MoreMath;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

@TeleOp(name="Odometry Test", group="Odometry")
public class Teleop extends OpMode {
    Hardware robot = new Hardware();

    Joystick leftStick;
    Joystick rightStick;

    double setPos = .2;
    double min = .2;
    double max = .8;
    double moveAmount = 0.05;

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

        leftStick = new Joystick(gamepad1, Joystick.Stick.LEFT, 2);
        rightStick = new Joystick(gamepad1, Joystick.Stick.RIGHT, 2);

        robot.drivetrain.setGoal(0, 1000, Math.toRadians(0));

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
        robot.drivetrain.drive(leftStick.getX(1), leftStick.getY(1),
                rightStick.getX(1));

        if (gamepad1.left_trigger > 0) {
            double wobblePower = gamepad1.left_trigger;
            robot.wobbleLeft.setPower(wobblePower);
            robot.wobbleRight.setPower(wobblePower);
        } else if (gamepad1.right_trigger > 0) {
            double wobblePower = -gamepad1.right_trigger;
            robot.wobbleLeft.setPower(wobblePower);
            robot.wobbleRight.setPower(wobblePower);
        } else {
            robot.wobbleLeft.setPower(0);
            robot.wobbleRight.setPower(0);
        }

        if (gamepad1.left_bumper) {
            robot.shooter.setVelocity(robot.desiredSpeed, AngleUnit.DEGREES);
        } else {
            robot.shooter.setVelocity(0);
        }

        if (gamepad1.right_bumper) {
            robot.intake.setPower(1.0);
        } else {
            robot.intake.setPower(-.1);
        }

        if (gamepad2.dpad_up) {
            robot.wobbleLift.setPower(.8);
        } else if (gamepad2.dpad_down) {
            robot.wobbleLift.setPower(-.2);
        }

        if (gamepad2.y) {
            robot.indexer.setPosition(setPos -= moveAmount);
        } else if (gamepad2.a) {
            robot.indexer.setPosition(setPos += moveAmount);
        }

        if (gamepad2.b) {
            robot.pusher.setPosition(1.0);
        } else {
            robot.pusher.setPosition(0);
        }

        if (robot.verbose) {
            verboseOutput();
        }

        //robot.wobbleLift.setPower(-gamepad1.right_stick_y);
        robot.wobbleArm.setPower(-gamepad2.right_stick_y);
        robot.shooterLift.setPower(-gamepad2.left_stick_y);

        //robot.wobbleLift.setPower(robot.wobbleLiftController.run(robot.wobbleLiftEnc.getDistance()));
        //robot.wobbleArm.setPower(robot.wobbleArmController.run(robot.wobbleArmEnc.getDistance()));
        //robot.shooterLift.setPower(robot.shooterLiftController.run(robot.shooterLiftEnc.getDistance()));

        //robot.odometry.writePoseToFile();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.odometry.setOffset(0, 0, 0);
    }

    private void verboseOutput() {
        // Send Odometry info
        telemetry.addLine("Odometry:");
        telemetry.addData("Theta", MoreMath.round(robot.odometry.getHeadingTheta(), 2));
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
        telemetry.addData("X", leftStick.getX(1));
        telemetry.addData("Y", leftStick.getY(1));
        telemetry.addLine("rightStick:");
        telemetry.addData("X", rightStick.getX(1));
        telemetry.addData("Y", rightStick.getY(1));
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
        telemetry.addData("shooterLift power", robot.shooterLift.getPower());
        telemetry.addData("shooterLift position", robot.shooterLiftEnc.getDistance());
        telemetry.addData("shooterLift port", robot.shooterLift.getPortNumber());
        telemetry.addLine();
        telemetry.addData("shooter power", robot.shooter.getPower());
        telemetry.addData("shooter degrees/time", robot.shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter degrees/time", robot.shooter.getVelocity(AngleUnit.DEGREES) / (360) * 60);
        telemetry.addData("shooter port", robot.shooter.getPortNumber());
        telemetry.addLine();
        telemetry.addData("Wobble lift power", robot.wobbleLift.getPower());
        telemetry.addData("Wobble lift port", robot.wobbleLift.getPortNumber());
        telemetry.addData("Wobble lift encoder", robot.wobbleLiftEnc.getDistance());
        telemetry.addLine();
        telemetry.addData("Wobble arm power", robot.wobbleArm.getPower());
        telemetry.addData("Wobble arm port", robot.wobbleArm.getPortNumber());
        telemetry.addData("Wobble arm encoder", robot.wobbleArmEnc.getDistance());
        telemetry.addLine("");
        telemetry.addLine();

        // Send Servo info
        telemetry.addLine("Servos:");
        telemetry.addData("intake power", robot.intake.getPower());
        telemetry.addData("intake port", robot.intake.getPortNumber());
        telemetry.addLine();
        telemetry.addData("feeder power", robot.feeder.getPower());
        telemetry.addData("feeder port", robot.feeder.getPortNumber());
        telemetry.addLine();
        telemetry.addData("wobbleLeft power", robot.wobbleLeft.getPower());
        telemetry.addData("wobbleLeft port", robot.wobbleLeft.getPortNumber());
        telemetry.addData("wobbleRight power", robot.wobbleRight.getPower());
        telemetry.addData("wobbleRight port", robot.wobbleRight.getPortNumber());
        telemetry.addLine();
        telemetry.addData("pusher position", robot.pusher.getPosition());
        telemetry.addData("pusher port", robot.pusher.getPortNumber());
        telemetry.addLine();
        telemetry.addData("indexer position", robot.indexer.getPosition());
        telemetry.addData("indexer port", robot.indexer.getPortNumber());
        telemetry.addLine();
        telemetry.addData("sounderArm position", robot.sounderArm.getPosition());
        telemetry.addData("sounderArm port", robot.sounderArm.getPortNumber());
        telemetry.addLine("");
        telemetry.addLine();

        // Send Sensor info
        telemetry.addLine("Sensors:");
        telemetry.addData("sounder distance mm", robot.sounder.getDistance(DistanceUnit.MM));
        telemetry.addLine("");
        telemetry.addLine();
    }
}
