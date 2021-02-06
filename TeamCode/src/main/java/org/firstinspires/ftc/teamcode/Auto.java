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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

    double distTol = .1;

    double sounderHeight = 185;

    double noRing = sounderHeight - 0;
    double singleRing = sounderHeight - 20;
    double fourRing = sounderHeight - 80;

    WobbleDepot wobbleDepot;

    @Override
    public void runOpMode() {
        /** INIT */
        robot.init(hardwareMap);

        robot.odometry.setOffset((140 + 225), 225-190, Math.toRadians(90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.indexer.setPosition(.8);

        /** WAIT FOR START */
        waitForStart();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        runtime.reset();

        /** RUN OTHER STUFF*/

        //goodWait(2000);

        robot.sounderArm.setPosition(.8);

        goToGoal(400, 600, Math.toRadians(90));

        goToGoal(480, 1135, Math.toRadians(0));

        goodWait(1000);

        robot.wobbleArmController.setTarget(90); //TODO: tune

        double dist = robot.sounder.getDistance(DistanceUnit.MM);

        double numrings = (sounderHeight - dist)/20;

        if (numrings < 1 - distTol) {
            telemetry.addData("No Rings", dist);
            telemetry.addData("", numrings);
            telemetry.update();

            wobbleDepot = WobbleDepot.FRONT;
        } else if (numrings < 2 - distTol) {
            telemetry.addData("1 Rings", dist);
            telemetry.addData("", numrings);
            telemetry.update();

            wobbleDepot = WobbleDepot.MIDDLE;
        } else {
            telemetry.addData("4 Rings", dist);
            telemetry.addData("", numrings);
            telemetry.update();

            wobbleDepot = WobbleDepot.BACK;
        }

        robot.sounderArm.setPosition(.2);

        robot.indexer.setPosition(.39); //TODO: tune

        goToDepot(wobbleDepot);

        robot.shooter.setVelocity(robot.desiredSpeed); //TODO: tune
        robot.shooterLiftController.setTarget(robot.shooterAngle); //TODO: tune

        /**goToGoal(1280, 1000, Math.toRadians(15));

        robot.pusher.setPosition(0);

        goodWait(100);

        robot.pusher.setPosition(.22);

        robot.indexer.setPosition(.31); //TODO: tune

        //goodWait(100);

        goToGoal(1450, 1000, Math.toRadians(15));

        goodWait(300); //MARK < Search to change indexer move times

        robot.pusher.setPosition(0);

        goodWait(100);

        robot.pusher.setPosition(.21);

        robot.indexer.setPosition(.23); //TODO: tune

        //goodWait(100);

        goToGoal(1610, 1000, Math.toRadians(15));

        goodWait(300); //MARK

        robot.pusher.setPosition(0);

        goodWait(100);

        robot.pusher.setPosition(.22);

        //goodWait(100);

        robot.shooter.setVelocity(0);

        robot.indexer.setPosition(.8);

        robot.shooterLiftController.setTarget(0);

        robot.wobbleLeft.setPower(1);
        robot.wobbleRight.setPower(1);

        goToGoal(1327, 960, Math.toRadians(0)); //TODO: tune
        goToGoal(1327, 773, Math.toRadians(0));

        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        goToGoal(480, 900, Math.toRadians(180));

        goToDepotTwo(wobbleDepot);

        robot.wobbleLeft.setPower(-1);
        robot.wobbleRight.setPower(-1);

        goodWait(250); */

        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        goodWait(4000);

        //goToGoal(600, 2000, Math.toRadians(180));

        robot.odometry.writePoseToFile();
    }

    private void myIdle() {
        robot.shooterLift.setPower(robot.shooterLiftController.run(robot.shooterLiftEnc.getDistance()));
        robot.wobbleLift.setPower(robot.wobbleLiftController.run(robot.wobbleLiftEnc.getDistance()));
        robot.wobbleArm.setPower(robot.wobbleArmController.run(robot.wobbleArmEnc.getDistance()));

        teleDump();
    }

    public void waitForAuto() {
        ElapsedTime timer = new ElapsedTime();
        boolean timeout = false;

        do {
            robot.drivetrain.updateAutoDrive();

            if ((robot.drivetrain.isRunning())) {
                timer.reset();
            } else if (timer.milliseconds() > 750) {
                timeout = true;
            }
            myIdle();
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
        }
    }

    private enum WobbleDepot {
        FRONT, MIDDLE, BACK
    }

    private void goToDepot(WobbleDepot depot) {
        if (depot == WobbleDepot.FRONT) {
            goToGoal(480, 2280, Math.toRadians(-45));
            robot.wobbleLeft.setPower(-1);
            robot.wobbleRight.setPower(-1);
            goodWait(250);
            robot.wobbleArmController.setTarget(0);
            goToGoal(560, 2400, Math.toRadians(-180));
            goToGoal(700, 1900, Math.toRadians(-180));
        } else if (depot == WobbleDepot.MIDDLE) {
            goToGoal(480, 2900, Math.toRadians(45));
            robot.wobbleLeft.setPower(-1);
            robot.wobbleRight.setPower(-1);
            goodWait(250);
            robot.wobbleArmController.setTarget(0);
            goToGoal(480, 2900, Math.toRadians(180));
            goToGoal(400, 1900, Math.toRadians(180));
        } else if (depot == WobbleDepot.BACK) {
            goToGoal(400, 3100, Math.toRadians(-90));
            robot.wobbleLeft.setPower(-1);
            robot.wobbleRight.setPower(-1);
            goodWait(250);
            robot.wobbleArmController.setTarget(0);
            goToGoal(700, 1900, Math.toRadians(-180));
        }
    }

    private void goToDepotTwo(WobbleDepot depot) {
        if (depot == WobbleDepot.FRONT) {
            goToGoal(480, 1750, Math.toRadians(180+45));
        } else if (depot == WobbleDepot.MIDDLE) {
            goToGoal(480, 2320, Math.toRadians(180-45));
        } else if (depot == WobbleDepot.BACK) {
            goToGoal(480, 3100, Math.toRadians(180+90));
        }
    }

    private void teleDump() {
        telemetry.addData("Shooter Speed :", robot.shooter.getVelocity());
        telemetry.addData("Ring maybe :", wobbleDepot);
        telemetry.update();
    }
}
