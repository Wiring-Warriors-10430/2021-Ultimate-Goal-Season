package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Odometry Calibration", group="Odometry")
public class OdometerCalibration extends OpMode {
    Hardware robot = new Hardware();

    //IMU Sensor
    BNO055IMU imu;

    final double PIVOT_SPEED = 0.3;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_MM = (8192d / (38d * Math.PI));

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    double wheelBaseSeparation;
    double verticalEncoderTickOffsetPerDegree;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private boolean moving = true;
    private boolean first = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (moving) {
            //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
            if (getZAngle() < 90) {
                robot.frontRightDrive.setPower(-PIVOT_SPEED);
                robot.rearRightDrive.setPower(-PIVOT_SPEED);
                robot.frontLeftDrive.setPower(PIVOT_SPEED);
                robot.rearLeftDrive.setPower(PIVOT_SPEED);
                if(getZAngle() < 60) {
                    setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
                }else{
                    setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
                }

                if (robot.verbose) {
                    telemetry.addLine("Turning...");
                }
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            } else {
                //Stop the robot
                setPowerAll(0, 0, 0, 0);
                if (first) {
                    timer.reset();
                    first = false;
                }
                if (timer.milliseconds() < 1500){
                    if (robot.verbose) {
                        telemetry.addData("Stopping", timer.milliseconds());
                    }
                    telemetry.addData("IMU Angle", getZAngle());
                    telemetry.update();
                } else {
                    moving = false;
                    first = true;
                }
            }
        } else {
            if (first) {
                //Record IMU and encoder values to calculate the constants for the global position algorithm
                double angle = getZAngle();

                /*
                Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
                Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
                THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
               */
                double encoderDifference = Math.abs(robot.left.getTicks()) + (Math.abs(robot.right.getTicks()));

                verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

                wheelBaseSeparation = ((2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI* COUNTS_PER_MM));

                horizontalTickOffset = robot.center.getDistance()/Math.toRadians(angle);

                //Write the constants to text files
                ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
                ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

                first = false;
            }

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            if (robot.verbose) {
                //Display raw values
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.addData("Vertical Left Position", -robot.left.getTicks());
                telemetry.addData("Vertical Right Position", robot.right.getTicks());
                telemetry.addData("Horizontal Position", robot.center.getTicks());
                telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);
            }

            //Update values
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return -(imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.frontRightDrive.setPower(rf);
        robot.rearRightDrive.setPower(rb);
        robot.frontLeftDrive.setPower(lf);
        robot.rearLeftDrive.setPower(lb);
    }
}
