## Welcome!

This is the code repository for the Wiring Warriors team 10430's FIRST Tech Challenge Ultimate Goal 2020-2021 competition season. Here you will find all of the actual code used on this years robot! For more information about our robot and season, see our [engineering notebook](https://www.youtube.com/watch?v=xvFZjo5PgG0), and [portfolio](https://www.youtube.com/watch?v=xvFZjo5PgG0).

## Navigation

Team code can be found in [TeamCode/src/main/java/org/firstinspires/ftc/teamcode](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode). Here you will find our libraries directory, [lib/](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib), which includes all of our custom written convivence classes, such as Odometry tracking, PID controller, an EasyPID handler to make FIRST's provided PID on the REV hubs easier to use, extra math functions, a dedicated Encoder class to simplify the process of getting Encoder values converted to real-world units, and much more. In the root of [TeamCode/src/main/java/org/firstinspires/ftc/teamcode](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode) you will find all of our standard code. See the [readme](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md) for more information.

## Helpful Links

Our single Auto for driver convenience: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto.java)

Teleop with minor driver enhancements and full closed-loop PID control: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Teleop.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Teleop.java)

Hardware class showing off how PID controllers and Encoders can be easily made for use in Teleop and Auto: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hardware.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hardware.java)

Encoder class, used everywhere on our robot: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Encoder.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Encoder.java)

Joystick class used to double-square joysticks, both to make them less sensitive, and return maximum values at the corners (thanks to [Renaud Bédard](http://theinstructionlimit.com/squaring-the-thumbsticks) for the C code we ported to java): [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Joystick.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Joystick.java)

Meccanum drive class with field-centered controls to make auto super easy to work with (see [auto](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto.java)): [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MeccanumDrivetrain.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MeccanumDrivetrain.java)

More Math since Java math just isn't like python math :wink:: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MoreMath.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MoreMath.java)

Odometry for field position tracking: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Odometry.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Odometry.java)

PIDF controller used for auto, as well as just about every mechanism on the robot with an encoder (best used with the [Encoder](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Encoder.java) class): [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/PIDFController.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/PIDFController.java)
