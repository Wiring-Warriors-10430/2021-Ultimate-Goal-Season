package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private double kP = 1;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private double rampDist = 0;

    private boolean enabled;

    private double previous_error = 0;
    private double integral = 0;

    private double setpoint = 0;
    private double startPoint = 0;
    private double lastTime = 0;

    private double output = 0;

    private double tolerance;

    private boolean running;

    private boolean bool = true;

    private ElapsedTime time = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF, double tolerance, double rampUpDist) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.rampDist = rampUpDist;

        this.tolerance = tolerance;

        enabled = true;
    }

    public double run(double currentPosition) {
        if (enabled) {
            if (bool) {
                startPoint = currentPosition;
                bool = false;
            }

            if (!(setpoint - tolerance <= currentPosition && currentPosition <= setpoint + tolerance)) {
                running = true;

                double ramp = rampIGuess(currentPosition);
                double pid = pidIGuess(currentPosition);

                if (ramp == -9999999999999999999.123456789123456789123456789123456789123456789123456789123456789) {
                    return pid;
                }  else if (pid > ramp) {
                    return ramp;
                } else {
                    return pid;
                }

                //return pid;
            } else {
                running = false;
                return 0;
            }
        } else {
            return 0;
        }

    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getOutput() {
        return output;
    }

    public void setTarget(double ticks) {
        bool = true;

        setpoint = ticks;
    }

    public double getTarget() {
        return setpoint;
    }

    public boolean isRunning() {
        return running;
    }

    private double pidIGuess(double currentPosition) {
        double dt = time.milliseconds() - lastTime;

        double error = setpoint - currentPosition;
        integral = integral + error * dt;
        double derivative = (error - previous_error) / dt;
        output = kP * error + kI * integral + kD * derivative;
        previous_error = error;
        lastTime = time.milliseconds();

        return output;
    }

    private double rampIGuess(double currentPosition) {
        double distTraveled = Math.abs(currentPosition - startPoint);

        if (distTraveled > rampDist) {
            return -9999999999999999999.123456789123456789123456789123456789123456789123456789123456789;
        } else {
            return distTraveled/rampDist + kF;
        }
    }
}
