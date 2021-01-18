package org.firstinspires.ftc.teamcode.lib;

public class Vector2 {
    public double x;
    public double y;

    public static Vector2 zero = new Vector2(0,0);

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 lerp(Vector2 a, Vector2 b, double factor) {
        return new Vector2(lerp(a.x, b.x, factor), lerp(a.y, b.y, factor));
    }

    public static double lerp(double a, double b, double c) {
        return a + (b - a) * c;
    }

    public double length() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
}
