package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Joystick {
    private Gamepad gamepad;
    private Stick side;
    private double pow;

    public enum Stick {
        LEFT,
        RIGHT
    }

    public Joystick(Gamepad pad, Stick stick) {
        gamepad = pad;
        side = stick;
        pow = 1;
    }

    public Joystick(Gamepad pad, Stick stick, double power) {
        gamepad = pad;
        side = stick;
        pow = power;
    }

    public double getRawX() {
        double value;

        if (side == Stick.LEFT) {
            value = gamepad.left_stick_x;
        } else {
            value = gamepad.right_stick_x;
        }

        if (value < 0 && pow % 2 == 0) {
            return - Math.pow(value, pow);
        } else {
            return Math.pow(value, pow);
        }
    }

    public double getRawY() {
        double value;

        if (side == Stick.LEFT) {
            value = - gamepad.left_stick_y;
        } else {
            value = - gamepad.right_stick_y;
        }

        if (value < 0 && pow % 2 == 0) {
            return - Math.pow(value, pow);
        } else {
            return Math.pow(value, pow);
        }
    }

    public Vector2 getRaw() {
        return new Vector2(getRawX(), getRawY());
    }

    public double getX() {
        return circleToSquare(getRaw()).x;
    }

    public double getY() {
        return circleToSquare(getRaw()).y;
    }

    public Vector2 getVector2() {
        return circleToSquare(getRaw());
    }

    public double getX(double roundness) {
        return circleToSquare(getRaw(), roundness).x;
    }

    public double getY(double roundness) {
        return circleToSquare(getRaw(), roundness).y;
    }

    public Vector2 getVector2(double roundness) {
        return circleToSquare(getRaw(), roundness);
    }

    /**
     *    Credit: Renaud Bédard http://theinstructionlimit.com/squaring-the-thumbsticks
     */
    private static Vector2 circleToSquare(Vector2 point) {
        return circleToSquare(point, 0);
    }

    private static Vector2 circleToSquare(Vector2 point, double innerRoundness) {
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
            // X- wall
        } else if (angle > 3 * PiOverFour && angle <= 5 * PiOverFour) {
            squared = new Vector2(point.x * (float)(-1 / Math.cos(angle)), point.y * (float)(-1 / Math.cos(angle)));
        // Y- wall
        } else if (angle > 5 * PiOverFour && angle <= 7 * PiOverFour) {
            squared = new Vector2(point.x * (float) (-1 / Math.sin(angle)), point.y * (float) (-1 / Math.sin(angle)));
        } else {
            squared = Vector2.zero;
        }
        // Early-out for a perfect square output
        if (innerRoundness == 0) {
            return squared;
        }

        // Find the inner-roundness scaling factor and LERP
        double length = point.length();
        double factor = (float) Math.pow(length, innerRoundness);
        return Vector2.lerp(point, squared, factor);
    }

}
