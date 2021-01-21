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

    @Override
    public void runOpMode() {
        /** INIT */
        robot.init(hardwareMap);

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
        goToGoal(0, 1000, Math.toRadians(0));

        goToGoal(1000, 1000, Math.toRadians(0));

        /**robot.drivetrain.linearGoToGoal(0, 1000, Math.toRadians(-90));

        robot.drivetrain.linearGoToGoal(1000, 1000, Math.toRadians(-180));

        robot.drivetrain.linearGoToGoal(1000, 0, Math.toRadians(-180));

        robot.drivetrain.linearGoToGoal(1000, 0, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(0, 0, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(0, 1000, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(1000, 1000, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(1000, 0, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(0, 0, Math.toRadians(-270));

        robot.drivetrain.linearGoToGoal(1000, 1000, Math.toRadians(0));

        robot.drivetrain.linearGoToGoal(0, 1000, Math.toRadians(180));

        robot.drivetrain.linearGoToGoal(1000, 0, Math.toRadians(0));

        robot.drivetrain.linearGoToGoal(500, 500, Math.toRadians(720));*/
    }

    public void waitForAuto() {
        ElapsedTime timer = new ElapsedTime();
        boolean timeout = false;

        do {
            // Send Odometry info
            telemetry.addLine("Odometry:");
            telemetry.addData("Theta", robot.odometry.getHeadingTheta());
            telemetry.addData("X", robot.odometry.getX());
            telemetry.addData("Y", robot.odometry.getY());
            telemetry.addData("Left", robot.left.getDistance());
            telemetry.addData("Right", robot.right.getDistance());
            telemetry.addData("Center", robot.center.getDistance());
            telemetry.addData("Conversion", robot.odometerToMM);
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();

            robot.drivetrain.updateAutoDrive();

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
}
