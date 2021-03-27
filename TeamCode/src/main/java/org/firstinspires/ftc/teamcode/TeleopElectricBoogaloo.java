package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.MoreMath;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

@TeleOp(name="Teleleleleleleop", group="Odometry")
public class Teleop extends OpMode {
    Hardware robot = new Hardware();

    Joystick leftStick;
    Joystick rightStick;

    floorPos setPos = floorPos.INTAKE;
    double min = .2;
    double max = .8;
    double moveAmount = 0.01;

    boolean shooterDown = true;
    boolean debounce = false;

    boolean autoShooterDown = true;
    boolean autoDebounce = false;

    boolean autoSpeed = false;

    boolean debounce2 = false;
    enum floorPos {
        TOP,
        MID,
        BOTTOM,
        INTAKE
    }

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

        if (gamepad2.left_bumper) {
            autoSpeed = false;
            robot.shooterVelocityController.setTarget(robot.desiredSpeed);
        } else if (gamepad2.back) {
            autoSpeed = true;
            robot.shooterVelocityController.setTarget(robot.autoDesiredSpeed);
        } else {
            robot.shooterVelocityController.setTarget(0);
        }

        if (gamepad1.left_bumper) {
            robot.intakeFloor.setPosition(.48);
            robot.intake.setPower(1.0);
            if (setPos == floorPos.INTAKE) {
                robot.feeder.setPower(1.0);
            } else {
                robot.feeder.setPower(-.1);
            }
        } else if (gamepad1.right_bumper) {
            robot.intakeFloor.setPosition(.48);
            robot.intake.setPower(-1.0);
            robot.feeder.setPower(-1.0);
        } else {
            robot.intakeFloor.setPosition(.52);
            robot.intake.setPower(-.1);
            robot.feeder.setPower(-.1);
        }

        /**if (gamepad2.dpad_up) {
            robot.wobbleLift.setPower(.8);
        } else if (gamepad2.dpad_down) {
            robot.wobbleLift.setPower(-.2);
        } else {
            robot.wobbleLift.setPower(0);
        }*/

        if (gamepad2.dpad_up) {
            robot.wobbleLiftController.setTarget(300); //374.65
        } else if (gamepad2.dpad_down) {
            robot.wobbleLiftController.setTarget(0);
        }

        if (gamepad2.dpad_right) {
            robot.wobbleArmController.setTarget(90);
        } else if (gamepad2.dpad_left) {
            robot.wobbleArmController.setTarget(0);
        }

        /**if (gamepad2.y) {
            setPos = MoreMath.clamp(setPos-moveAmount, .2, .8);
            robot.indexer.setPosition(setPos);
        } else if (gamepad2.a) {
            setPos = MoreMath.clamp(setPos+moveAmount, .2, .8);
            robot.indexer.setPosition(setPos);
        } else {
            setPos = MoreMath.clamp(setPos, .2, .8);
            robot.indexer.setPosition(setPos);
        }*/

        if (gamepad2.y && !debounce2) {
            debounce2 = true;
            if (setPos == floorPos.INTAKE) {
                setPos = floorPos.BOTTOM;
                robot.indexer.setPosition(robot.indexerLow);
            } else if (setPos == floorPos.BOTTOM) {
                setPos = floorPos.MID;
                robot.indexer.setPosition(robot.indexerMid);
            } else if (setPos == floorPos.MID) {
                setPos = floorPos.TOP;
                robot.indexer.setPosition(robot.indexerHigh);
            }
        } else if (!gamepad2.y && debounce2) {
            debounce2 = false;
        }

        if (gamepad2.a && robot.shooterLiftController.getTarget() <= 2) {
            setPos = floorPos.INTAKE;
            robot.indexer.setPosition(robot.indexerLowest);
        }

        if (gamepad2.b ) {
            if (autoSpeed) {
                if (robot.autoShooterAtSpeed()) {
                    robot.pusher.setPosition(0);
                }
            } else {
                if (robot.shooterAtSpeed()) {
                    robot.pusher.setPosition(0);
                }
            }
        } else {
            robot.pusher.setPosition(.22);
        }

        if (gamepad2.x && !debounce) {
            shooterDown = !shooterDown;
            debounce = true;
        } else if (!gamepad2.x && debounce) {
            debounce = false;
        }

        if (shooterDown) {
            robot.shooterLiftController.setTarget(0);
        } else {
            robot.shooterLiftController.setTarget(robot.shooterAngle);
            if (floorPos.INTAKE == setPos) {
                setPos = floorPos.BOTTOM;
                robot.indexer.setPosition(.38);
            }
        }

        /**if (gamepad2.start && !autoDebounce) {
            autoShooterDown = !autoShooterDown;
            autoDebounce = true;
        } else if (!gamepad2.start && autoDebounce) {
            autoDebounce = false;
        }*/

        if (robot.verbose) {
            verboseOutput();
        }

        if (gamepad1.a) {
            robot.drivetrain.updateAutoDrive();
        } else {
            robot.drivetrain.drive(leftStick.getX(1), leftStick.getY(1),
                    rightStick.getX(1));
        }

        //robot.wobbleLift.setPower(-gamepad1.right_stick_y);
        //robot.wobbleArm.setPower(-gamepad2.right_stick_y);
        //robot.shooterLift.setPower(-gamepad2.left_stick_y);

        robot.wobbleLift.setPower(robot.wobbleLiftController.run(robot.wobbleLiftEnc.getDistance()));
        robot.wobbleArm.setPower(robot.wobbleArmController.run(robot.wobbleArmEnc.getDistance()));
        robot.shooterLift.setPower(robot.shooterLiftController.run(robot.shooterLiftEnc.getDistance()));
        robot.shooter.setPower(robot.shooterVelocityController.run(robot.shooter.getVelocity()));

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
        telemetry.addData("angle", robot.shooterLiftEnc.getDistance());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();

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
        telemetry.addData("shooter degrees/time", robot.shooter.getVelocity());
        telemetry.addData("shooter RPM", (robot.shooter.getVelocity()/28)*60);
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
        telemetry.addData("high sounder distance mm", robot.highSounder.getDistance(DistanceUnit.MM));
        telemetry.addData("low sounder distance mm", robot.lowSounder.getDistance(DistanceUnit.MM));
        telemetry.addLine("");
        telemetry.addLine();
    }
}
