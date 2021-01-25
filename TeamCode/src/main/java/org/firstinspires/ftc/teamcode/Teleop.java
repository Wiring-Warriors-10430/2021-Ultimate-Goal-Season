package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

@TeleOp(name="Odometry Test", group="Odometry")
public class Teleop extends OpMode {
    Hardware robot = new Hardware();

    Joystick leftStick;
    Joystick rightStick;

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

        robot.odometry.setOffset(0l, 0, Math.toRadians(90));

        leftStick = new Joystick(gamepad1, Joystick.Stick.LEFT, 2);
        rightStick = new Joystick(gamepad1, Joystick.Stick.RIGHT, 2);

        robot.drivetrain.setGoal(500, 1000, Math.toRadians(90));

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

        if (gamepad1.a) {
            robot.drivetrain.updateAutoDrive();
        }

        if (robot.verbose) {
            verboseOutput();
        }

        robot.odometry.writePoseToFile();
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
        telemetry.addData("Theta", roundAvoid(robot.odometry.getHeadingTheta(), 2));
        telemetry.addData("X", roundAvoid(robot.odometry.getX(), 2));
        telemetry.addData("Y", roundAvoid(robot.odometry.getY(), 2));
        telemetry.addData("Left", robot.left.getDistance());
        telemetry.addData("Right", robot.right.getDistance());
        telemetry.addData("Center", robot.center.getDistance());
        telemetry.addData("Conversion", robot.odometerToMM);
        telemetry.addLine("");

        // Send Joystick Info
        telemetry.addLine("Sticks");
        telemetry.addLine("leftStick:");
        telemetry.addData("X", leftStick.getX(1));
        telemetry.addData("Y", leftStick.getY(1));
        telemetry.addLine("rightStick:");
        telemetry.addData("X", rightStick.getX(1));
        telemetry.addData("Y", rightStick.getY(1));

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
        telemetry.addData("shooterLift power", robot.shooterLift.getPower());
        telemetry.addData("shooterLift position", robot.shooterLift.getCurrentPosition());
        telemetry.addData("shooterLift target position", robot.shooterLift.getTargetPosition());
        telemetry.addData("shooterLift port", robot.shooterLift.getPortNumber());
        telemetry.addData("shooter power", robot.shooter.getPower());
        telemetry.addData("shooter degrees/time", robot.shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shooter port", robot.shooter.getPortNumber());
        telemetry.addLine("");

        // Send Servo info
        telemetry.addLine("Servos:");
        telemetry.addData("intake power", robot.intake.getPower());
        telemetry.addData("intake port", robot.intake.getPortNumber());
        telemetry.addData("feeder power", robot.feeder.getPower());
        telemetry.addData("feeder port", robot.feeder.getPortNumber());
        telemetry.addData("wobbleLeft power", robot.wobbleLeft.getPower());
        telemetry.addData("wobbleLeft port", robot.wobbleLeft.getPortNumber());
        telemetry.addData("wobbleRight power", robot.wobbleRight.getPower());
        telemetry.addData("wobbleRight port", robot.wobbleRight.getPortNumber());
        telemetry.addData("pusher position", robot.pusher.getPosition());
        telemetry.addData("pusher port", robot.pusher.getPortNumber());
        telemetry.addData("indexer position", robot.indexer.getPosition());
        telemetry.addData("indexer port", robot.indexer.getPortNumber());
        telemetry.addData("sounderArm position", robot.sounderArm.getPosition());
        telemetry.addData("sounderArm port", robot.sounderArm.getPortNumber());
        telemetry.addLine("");

        // Send Sensor info
        telemetry.addLine("Sensors:");
        telemetry.addData("sounder distance mm", robot.sounder.getDistance(DistanceUnit.MM));
        telemetry.addLine("");
    }

    public static double roundAvoid(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}
