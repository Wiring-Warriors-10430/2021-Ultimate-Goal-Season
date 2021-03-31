package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.lib.Joystick;
import org.firstinspires.ftc.teamcode.lib.MoreMath;
import org.firstinspires.ftc.teamcode.lib.Pose2D;

import java.util.List;

@TeleOp(name="Teleleleleleleop", group="_Robot")
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

    boolean debounce2 = false;
    enum floorPos {
        TOP,
        MID,
        BOTTOM,
        INTAKE
    }

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

        leftStick = new Joystick(gamepad1, Joystick.Stick.LEFT, 2);
        rightStick = new Joystick(gamepad1, Joystick.Stick.RIGHT, 2);

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
    public void start() { telemetry.log().clear();}

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
            robot.shooterVelocityController.setTarget(robot.desiredSpeed);
        } else if (gamepad2.back) {
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

        if (gamepad2.b && (robot.shooterAtSpeed())) {
            robot.pusher.setPosition(0);
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
        telemetry.addData("NavX Heading", robot.getHeading());
        telemetry.addLine();
        telemetry.addLine();

        // Dump some vision
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
        telemetry.addData("Stack", robot.measureStack());
    }
}
