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

        robot.odometry.setOffset((140 + 225), 225, Math.toRadians(90));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** WAIT FOR START */
        while (!isStarted()) { }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        runtime.reset();

        /** RUN OTHER STUFF*/

        //goodWait(2000);

        robot.sounderArm.setPosition(.8);

        goToGoal(400, 600, Math.toRadians(90));

        goToGoal(500, 1175, Math.toRadians(0));

        goodWait(1000);

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

        goToDepot(wobbleDepot);

        //TODO: Drop Wobble
        robot.wobbleArm.setPower(.5);

        goodWait(50);

        robot.wobbleArm.setPower(0);

        robot.wobbleLeft.setPower(-1);
        robot.wobbleRight.setPower(-1);

        goToGoal(480, 1700, Math.toRadians(0));
        goToGoal(1280, 1700, Math.toRadians(0));

        //TODO: Shoot first powershot
        robot.indexer.setPosition(.5);

        goodWait(50);

        robot.pusher.setPosition(1);

        goodWait(50);

        robot.pusher.setPosition(0);

        goodWait(50);


        goToGoal(1450, 1700, Math.toRadians(0));

        //TODO: Shoot second Powershot
        robot.indexer.setPosition(.35);

        goodWait(50);

        robot.pusher.setPosition(1);

        goodWait(50);

        robot.pusher.setPosition(0);

        goodWait(50);

        goToGoal(1610, 1700, Math.toRadians(0));

        //TODO: Shoot third Powershot
        robot.indexer.setPosition(.2);

        goodWait(50);

        robot.pusher.setPosition(1);

        goodWait(50);

        robot.pusher.setPosition(0);

        goodWait(50);

        robot.wobbleLeft.setPower(1);
        robot.wobbleRight.setPower(1);

        goToGoal(1150, 900, Math.toRadians(0));
        goToGoal(480, 900, Math.toRadians(180));

        //TODO: Pickup second Wobble
        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        goToDepotTwo(wobbleDepot);

        //TODO: Drop Second Wobble
        robot.wobbleLeft.setPower(-1);
        robot.wobbleRight.setPower(-1);

        goToGoal(480, 2000, Math.toRadians(180));

        robot.wobbleLeft.setPower(0);
        robot.wobbleRight.setPower(0);

        goodWait(5000);

        robot.odometry.writePoseToFile();
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

        }
    }

    private enum WobbleDepot {
        FRONT, MIDDLE, BACK
    }

    private void goToDepot(WobbleDepot depot) {
        if (depot == WobbleDepot.FRONT) {
            goToGoal(480, 2280, Math.toRadians(-45));
        } else if (depot == WobbleDepot.MIDDLE) {
            goToGoal(480, 2900, Math.toRadians(45));
        } else if (depot == WobbleDepot.BACK) {
            goToGoal(480, 3100, Math.toRadians(-90));
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
}
