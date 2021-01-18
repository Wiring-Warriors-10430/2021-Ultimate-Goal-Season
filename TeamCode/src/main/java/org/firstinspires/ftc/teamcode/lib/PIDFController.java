package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private double kP = 1;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private boolean enabled;

    private double previous_error = 0;
    private double integral = 0;

    private int setpoint = 0;
    private double lastTime = 0;

    private double output = 0;

    private ElapsedTime time = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        enabled = true;
    }

    public double run(double currentPosition) {
        double dt = time.milliseconds() - lastTime;

        double error = setpoint - currentPosition;
        integral = integral + error * dt;
        double derivative = (error - previous_error) / dt;
        output = kP * error + kI * integral + kD * derivative;
        previous_error = error;
        lastTime = time.milliseconds();

        return output;
    }

    public double getOutput() {
        return output;
    }

    public void setTarget(int ticks) {
        setpoint = ticks;
    }

    public int getTarget() {
        return setpoint;
    }
}
