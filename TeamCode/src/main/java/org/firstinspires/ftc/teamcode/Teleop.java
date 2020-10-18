package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class Teleop extends OpMode {
    Hardware robot = new Hardware();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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
        //TODO: Migrate this to it's own thread.
        robot.odometry.run();

        // Send telemetry message to signify robot running;
        telemetry.addData("Theta", robot.odometry.getRobotThetaRad());
        telemetry.addData("X", robot.odometry.getRobotX());
        telemetry.addData("Y", robot.odometry.getRobotY());
        telemetry.addData("Left", robot.left.getDistance());
        telemetry.addData("Right", robot.right.getDistance());
        telemetry.addData("Center", robot.center.getDistance());
        telemetry.addData("Conversion", robot.odometerToMM);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}