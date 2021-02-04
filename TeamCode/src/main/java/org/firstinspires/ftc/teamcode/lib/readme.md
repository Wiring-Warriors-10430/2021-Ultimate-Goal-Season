  
## Libraries

This is where you will find all of our custom convenience classes, which should be able to be copied into your own FIRST Tech Challenge projects. Below is an outline of each class in this directory. For more information on our main code, see  [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/new/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md).

## Outline

[EasyPID.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/EasyPID.java) : A simple class to make working with the REV hub's built-in PIDF loops clearer.

[Encoder.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Encoder.java) : Class built to make interfacing with robot encoders far easier. Includes full support for unity conversion, making your code elsewhere far simpler.

[Joystick.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Joystick.java) : Class which makes it easier to enhance joysticks for driver controls. Comes with an enumerator to set left vs. right stick, support for joysticks brought to any power (see [here](https://www.chiefdelphi.com/t/joystick-sensitivity/95510/8) for explanation), and squaring of the sticks to output values of one at 45 degree angles (thanks to [Renaud Bédard](http://theinstructionlimit.com/squaring-the-thumbsticks) for the C code we ported to Java).

[MeccanumDrivetrain.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MeccanumDrivetrain.java) : A simple container to allow us to access drive code in both Teleoperated and Autonomous modes. Includes field-centered drive based on Odometry heading.

[MoreMath.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/MoreMath.java) : Not all languages are created equal, and Java needed some more math functions. Includes rounding and clipping of values.

[Odometry.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Odometry.java) : A simple handler to make OdometryGlobalTracking.java easier to use.

[OdometryGlobalTracking.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/OdometryGlobalTracking.java) : A very trig-heavy class to solve the position in space of a holonomic drivetrain using a background thread. Uses three omni-wheels with encoders to replace drive encoders and make auto easy when used in conjunction with PID loops.

[PIDFController.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/PIDFController.java) : A simple class which runs a PIDF loop decoupled from the output of a motor port on a REV. Preferred by us for it's simplicity, and used in Autonomous to solve the robot's position on the field.

[Pose2D.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Pose2D.java) : A simple class to store position data. For example, to store a robot's x, y, and heading positions.

[Vector2.java](https://github.com/Wiring-Warriors-10430/2021-Ultimate-Goal-Season/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/Vector2.java) : A class Java doesn't include on it's own to store an x, y pair.
