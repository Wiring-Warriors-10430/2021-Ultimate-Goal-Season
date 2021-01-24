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

    double distTol = 2;

    double sounderHeight = 168;

    double noRing = sounderHeight - 0;
    double singleRing = sounderHeight - 20;
    double fourRing = sounderHeight - 80;

    @Override
    public void runOpMode() {
        /** INIT */
        robot.init(hardwareMap);

        //robot.odometry.setOffset((140 + 225), 225, Math.toRadians(90));

        robot.odometry.setOffset(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** WAIT FOR START */
        while (!isStarted()) {
            // Send Odometry info
            telemetry.addLine("Odometry:");
            telemetry.addData("Theta", robot.odometry.getHeadingTheta());
            telemetry.addData("X", robot.odometry.getX());
            telemetry.addData("Y", robot.odometry.getY());
            telemetry.addData("Left", robot.left.getDistance());
            telemetry.addData("Right", robot.right.getDistance());
            telemetry.addData("Center", robot.center.getDistance());
            telemetry.addData("Conversion", robot.odometerToMM);
            telemetry.addLine("");
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        runtime.reset();

        /** RUN OTHER STUFF*/

        goodWait(2000);

        robot.sounderArm.setPosition(.4);

        goToGoal(545, 1190, Math.toRadians(90));

        goodWait(1000);

        double dist = robot.sounder.getDistance(DistanceUnit.MM);

        if (dist > singleRing + distTol) {
            telemetry.addLine("no rings");
            telemetry.update();
        } else if (dist > fourRing + distTol) {
            telemetry.addLine("1 rings");
            telemetry.update();
        } else {
            telemetry.addLine("4 rings");
            telemetry.update();
        }

        goodWait(5000);

        robot.odometry.writePoseToFile();
    }

    public void waitForAuto() {
        ElapsedTime timer = new ElapsedTime();
        boolean timeout = false;

        do {
            robot.drivetrain.updateAutoDrive();

            // Send Odometry info
            telemetry.addLine("Odometry:");
            telemetry.addData("Theta", robot.odometry.getHeadingTheta());
            telemetry.addData("X", robot.odometry.getX());
            telemetry.addData("Y", robot.odometry.getY());
            telemetry.addData("Left", robot.left.getDistance());
            telemetry.addData("Right", robot.right.getDistance());
            telemetry.addData("Center", robot.center.getDistance());
            telemetry.addData("Conversion", robot.odometerToMM);
            telemetry.addLine("");
            telemetry.update();

            if ((robot.drivetrain.isRunning())) {
                timer.reset();
            } else if (timer.milliseconds() > 100) {
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
            // Send Odometry info
            telemetry.addLine("Odometry:");
            telemetry.addData("Theta", robot.odometry.getHeadingTheta());
            telemetry.addData("X", robot.odometry.getX());
            telemetry.addData("Y", robot.odometry.getY());
            telemetry.addData("Left", robot.left.getDistance());
            telemetry.addData("Right", robot.right.getDistance());
            telemetry.addData("Center", robot.center.getDistance());
            telemetry.addData("Conversion", robot.odometerToMM);
            telemetry.addLine("");
            telemetry.update();
        }
    }
}
