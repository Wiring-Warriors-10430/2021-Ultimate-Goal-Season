package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.InvalidObjectException;

public class Joystick {
    private Gamepad gamepad;
    private Stick side;

    public enum Stick {
        LEFT,
        RIGHT
    }

    public Joystick(Gamepad pad, Stick stick) {
        gamepad = pad;
        side = stick;
    }

    public double getRawX() {
        if (side == Stick.LEFT) {
            return gamepad.left_stick_x;
        } else {
            return gamepad.right_stick_x;
        }
    }

    public double getRawY() {
        if (side == Stick.LEFT) {
            return gamepad.left_stick_y;
        } else {
            return gamepad.right_stick_y;
        }
    }

    public Vector2 getRaw() {
        if (side == Stick.LEFT) {
            return new Vector2(gamepad.left_stick_x, gamepad.left_stick_y);
        } else {
            return new Vector2(gamepad.right_stick_x, gamepad.right_stick_y);
        }
    }

    public double getX() {
        return CircleToSquare(getRaw()).x;
    }

    public double getY() {
        return CircleToSquare(getRaw()).y;
    }

    public Vector2 getVector2() {
        return CircleToSquare(getRaw());
    }

    public double getX(double roundness) {
        return CircleToSquare(getRaw(), roundness).x;
    }

    public double getY(double roundness) {
        return CircleToSquare(getRaw(), roundness).y;
    }

    public Vector2 getVector2(double roundness) {
        return CircleToSquare(getRaw(), roundness);
    }

    /**
     *    Credit: Renaud Bédard http://theinstructionlimit.com/squaring-the-thumbsticks
     */
    private static Vector2 CircleToSquare(Vector2 point) {
        return CircleToSquare(point, 0);
    }

    private static Vector2 CircleToSquare(Vector2 point, double innerRoundness) {
        final double PiOverFour = Math.PI / 4;

        // Determine the theta angle
        double angle = Math.atan2(point.y, point.x) + Math.PI;

        Vector2 squared;
        // Scale according to which wall we're clamping to
        // X+ wall
        if (angle <= PiOverFour || angle > 7 * PiOverFour) {
            squared = new Vector2(point.x * (float) (1 / Math.cos(angle)), point.y * (float) (1 / Math.cos(angle)));
            // Y+ wall
        } else if (angle > PiOverFour && angle <= 3 * PiOverFour) {
            squared = new Vector2(point.x * (float) (1 / Math.sin(angle)), point.y * (float) (1 / Math.sin(angle)));
            ;
            // X- wall
        } else if (angle > 3 * PiOverFour && angle <= 5 * PiOverFour) {
            squared = new Vector2(point.x * (float)(-1 / Math.cos(angle)), point.y * (float)(1 / Math.sin(angle)));
        // Y- wall
        } else if (angle > 5 * PiOverFour && angle <= 7 * PiOverFour) {
            squared = new Vector2(point.x * (float) (-1 / Math.sin(angle)), point.y * (float) (1 / Math.sin(angle)));
        } else {
            squared = Vector2.zero;
        }
        // Early-out for a perfect square output
        if (innerRoundness == 0)
        return squared;

        // Find the inner-roundness scaling factor and LERP
        double length = point.Length();
        double factor = (float) Math.pow(length, innerRoundness);
        return Vector2.Lerp(point, squared, factor);
    }

}
