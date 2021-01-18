package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class EasyPID {
    private DcMotorEx motor;

    private double kP = 1;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private int t = 0;

    public EasyPID(DcMotorEx motor, double kP, double kI, double kD, double kF, int tolerance) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.t = tolerance;

        setup();
    }

    public EasyPID(DcMotorEx motor, double kP, double kI, double kD, double kF) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        setup();
    }

    public EasyPID(DcMotorEx motor, double kP, double kI, double kD) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        setup();
    }

    public EasyPID(DcMotorEx motor, double kP) {
        this.motor = motor;
        this.kP = kP;

        setup();
    }

    public EasyPID(DcMotorEx motor, double kP, double kF) {
        this.motor = motor;
        this.kP = kP;
        this.kF = kF;

        setup();
    }

    private void setup() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

        pidfCoefficients.p = kP;
        pidfCoefficients.i = kI;
        pidfCoefficients.d = kD;
        pidfCoefficients.f = kF;

        motor.setPIDFCoefficients(motor.getMode(), pidfCoefficients);

        motor.setTargetPositionTolerance(t);
    }

    public void enable() {
        motor.setMotorEnable();
    }

    public void disable() {
        motor.setMotorDisable();
    }

    public boolean getEnabled() {
        return motor.isMotorEnabled();
    }

    public void setTarget(int posOrRate) {
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(posOrRate);
        } else {
            motor.setVelocity(posOrRate);
        }
    }

    public double getTarget() {
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return motor.getTargetPosition();
        } else {
            return motor.getVelocity();
        }
    }

    public void setTargetTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    public int getTargetTolerance() {
        return motor.getTargetPositionTolerance();
    }
}
