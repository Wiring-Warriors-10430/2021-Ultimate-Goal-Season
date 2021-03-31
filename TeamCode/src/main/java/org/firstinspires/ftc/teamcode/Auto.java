/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto", group="Auto")
//@Disabled
public class Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Hardware robot = new Hardware();

    Hardware.Depot depot;

    static double pushRingDelay = 300;
    static double dropWobbleDelay = 250;
    static double moveIndexerDelay = 700;

    @Override
    public void runOpMode() {
        /** INIT */
        robot.init(hardwareMap);

        robot.intakeFloor.setPosition(.52);

        robot.odometry.setOffset(356, 229, Math.toRadians(180));

        while (robot.navxMicro.isCalibrating() && opModeIsActive()) {
            telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            idle();
        }

        robot.indexer.setPosition(robot.indexerLowest);

        robot.drivetrain.setVoltage(robot.voltageSensor.getVoltage());

        /** WAIT FOR START */
        while (!opModeIsActive()) {
            telemetry.log().add("Gyro Calibrated. Press Start.");
            telemetry.addData("Rings", robot.measureStack());
            telemetry.update();

            idle();
        }

        runtime.reset();

        /** RUN OTHER STUFF*/

        // Calculate Stack
        depot = robot.measureStack();

        if (depot == Hardware.Depot.BACK) {

        } else {
            // Move Wobble arm to score
            robot.wobbleArmController.setTarget(90);
        }

        // Indexer bottom
        robot.indexer.setPosition(robot.indexerLow);

        // Go to depot
        goToDepot(depot);

        // drop wobble
        robot.wobbleLeft.setPower(-1);
        robot.wobbleRight.setPower(-1);

        // wait for wobble to drop
        goodWait(dropWobbleDelay);

        // stow wobble arm
        robot.wobbleArmController.setTarget(0);

        // prep Shooter
        robot.shooterVelocityController.setTarget(robot.autoDesiredSpeed); //TODO: tune
        robot.shooterLiftController.setTarget(robot.autoShooterAngle); //TODO: tune

        // Move to shooting pos
        goToGoal(1450, 1400, Math.toRadians(10));

        // Push ring
        robot.pusher.setPosition(0);

        // Wait for pusher
        goodWait(pushRingDelay);

        // pusher in
        robot.pusher.setPosition(.22);

        // Indexer for second ring
        robot.indexer.setPosition(robot.indexerMid);

        // Second powershot
        goToGoal(1450, 1500, Math.toRadians(0));

        // Wait for indexer
        goodWait(moveIndexerDelay);

        // push Ring
        robot.pusher.setPosition(0);

        //Wait for pusher
        goodWait(pushRingDelay);

        // Pusher in
        robot.pusher.setPosition(.21);

        // Indexer for 3rd ring
        robot.indexer.setPosition(robot.indexerHigh);

        // Third powershot
        goToGoal(1450, 1500, Math.toRadians(-10));

        // Wait for indexer
        goodWait(moveIndexerDelay);

        // push ring
        robot.pusher.setPosition(0);

        // wait for pusher
        goodWait(pushRingDelay);

        // pusher in
        robot.pusher.setPosition(.22);

        // spin down
        robot.shooterVelocityController.setTarget(0);

        // Wobble to grab
        robot.wobbleArmController.setTarget(90);

        // Reset Indexer
        robot.indexer.setPosition(.8);

        // Shooter down
        robot.shooterLiftController.setTarget(0);

        // grab wobble
        robot.wobbleLeft.setPower(1);
        robot.wobbleRight.setPower(1);

        // Grab Wobble
        if (depot == Hardware.Depot.FRONT) {
            goToGoal(1250, 725, Math.toRadians(0));
        } else if (depot == Hardware.Depot.MIDDLE) {
            goToGoal(1390, 710, Math.toRadians(0));
        } else if (depot == Hardware.Depot.BACK) {
            goToGoal(1270, 725, Math.toRadians(0));
        }

        goodWait(500);

        // Hold wobble
        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        goToGoal(1350, 1250, Math.toRadians(0));

        // score second wobble
        goToDepotTwo(depot);

        // drop wobble
        robot.wobbleLeft.setPower(-1);
        robot.wobbleRight.setPower(-1);

        // wait for wobble
        goodWait(dropWobbleDelay);

        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        robot.wobbleArmController.setTarget(0);

        goToGoal(600, 1650, Math.toRadians(0));

        robot.odometry.writePoseToFile();
    }

    private void myIdle() {
        robot.shooterLift.setPower(robot.shooterLiftController.run(robot.shooterLiftEnc.getDistance()));
        robot.wobbleLift.setPower(robot.wobbleLiftController.run(robot.wobbleLiftEnc.getDistance()));
        robot.wobbleArm.setPower(robot.wobbleArmController.run(robot.wobbleArmEnc.getDistance()));

        robot.shooter.setPower(robot.shooterVelocityController.run(robot.shooter.getVelocity()));

        teleDump();
    }

    public void waitForAuto() {
        ElapsedTime timer = new ElapsedTime();
        boolean timeout = false;

        do {
            robot.drivetrain.updateAutoDrive();

            if ((robot.drivetrain.isRunning())) {
                timer.reset();
            } else if (timer.milliseconds() > 1500) {
                timeout = true;
            }
            myIdle();
            idle();
        } while (robot.drivetrain.isRunning() && opModeIsActive() && !timeout);
    }

    public void goToGoal(double x, double y, double heading) {
        robot.drivetrain.setGoal(x, y, heading);

        waitForAuto();
    }

    private void goodWait(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < time && opModeIsActive()) {
            myIdle();
            idle();
        }
    }

    private void goToDepot(Hardware.Depot depot) {
        if (depot == Hardware.Depot.FRONT) {
            // Go to depot one
            goToGoal(480, 1700, Math.toRadians(180+45));
        } else if (depot == Hardware.Depot.MIDDLE) {
            // Go to depot two
            goToGoal(500, 2380, Math.toRadians(180-45));
        } else if (depot == Hardware.Depot.BACK) {
            // Go to depot three
            goToGoal(800, 2750, Math.toRadians(-180));
            // Move Wobble arm to score
            robot.wobbleArmController.setTarget(90);
            goToGoal(600, 3000, Math.toRadians(-180+90));
        }
    }

    private void goToDepotTwo(Hardware.Depot depot) {
        if (depot == Hardware.Depot.FRONT) {
            // go to depot one a second time
            goToGoal(550, 1600, Math.toRadians(-90));
        } else if (depot == Hardware.Depot.MIDDLE) {
            // go ot depot two a second time
            goToGoal(480, 2100, Math.toRadians(180-45));
        } else if (depot == Hardware.Depot.BACK) {
            // go to depot three a second time
            goToGoal(480, 3000, Math.toRadians(-180+90));
        }
    }

    private void teleDump() {
        telemetry.addData("Shooter Speed :", robot.shooter.getVelocity());
        telemetry.addData("Measured Depot :", depot);
        telemetry.addData("Voltage", robot.drivetrain.getVoltage());
        telemetry.addData("theta happy", !robot.drivetrain.thetaAt());
        telemetry.addData("x happy", !robot.drivetrain.xAt());
        telemetry.addData("y happy", !robot.drivetrain.yAt());
        telemetry.update();
    }
}
