## TeamCode

This is where you will find all of our code. Below is an outline of each class in the root of this directory. For more information on our libraries, see [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/readme.md](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/readme.md).

## Outline

[Auto.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto.java) : The single Autonomous mode used this season.

[Hardware.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hardware.java) : A class to construct all of our hardware objects and certain globally necessary software objects. For e.g., [Odometry](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Odometry.java) and Motors.

[OdometerCalibration.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OdometerCalibration.java) : A simple Tele-op class to tune certain parameters of our physical [Odometry](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Odometry.java) setup.

[Teleop.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Teleop.java) : Our single Teleoperated class. Includes a few minor driver enhancements, and full closed-loop PID control on every mechanism with an [encoder](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Encoder.java).
