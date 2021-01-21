package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    private DcMotorEx motorPort;
    private int ticksPerRev;
    private double reduction;
    private double conversionFactor;
    private int reverse = 1;

    /**
     * @param motorObject      DcMotorEx object pointed at the same motor port as the encoder is plugged into.
     * @param ticksPerRev      Number of pulses output by the encoder per revolution of it's shaft BEFORE gearing.
     * @param reduction        The reduction between the encoder and input object.
     * @param conversionFactor The conversion factor to convert ticks AFTER reduction to output helpful units.
     */
    public Encoder(DcMotorEx motorObject, int ticksPerRev, double reduction, double conversionFactor) {
        motorPort = motorObject;
        this.ticksPerRev = ticksPerRev;
        this.reduction = reduction;
        this.conversionFactor = conversionFactor;

        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Encoder(DcMotorEx motorObject, int ticksPerRev, double reduction, ConversionFactor conversionFactor) {
        double cf;

        if (conversionFactor == ConversionFactor.DEGREE) {
            cf = (double) 1/360;
        } else if (conversionFactor == ConversionFactor.RADIAN) {
            cf = (double) 1 / (2 * Math.PI);
        } else {
            cf = (double) 1/ticksPerRev;
        }

        motorPort = motorObject;
        this.ticksPerRev = ticksPerRev;
        this.reduction = reduction;
        this.conversionFactor = cf;

        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Encoder(DcMotorEx motorObject, int ticksPerRev, double reduction, double conversionFactor, boolean reverse) {
        motorPort = motorObject;
        this.ticksPerRev = ticksPerRev;
        this.reduction = reduction;
        this.conversionFactor = conversionFactor;

        if (reverse) {
            this.reverse = -1;
        }

        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Encoder(DcMotorEx motorObject, int ticksPerRev, double reduction, ConversionFactor conversionFactor, boolean reverse) {
        double cf;

        if (conversionFactor == ConversionFactor.DEGREE) {
            cf = (double) 1/360;
        } else if (conversionFactor == ConversionFactor.RADIAN) {
            cf = (double) 1 / (2 * Math.PI);
        } else {
            cf = (double) 1/ticksPerRev;
        }

        motorPort = motorObject;
        this.ticksPerRev = ticksPerRev;
        this.reduction = reduction;
        this.conversionFactor = cf;

        if (reverse) {
            this.reverse = -1;
        }

        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getRawTicks() {
        return reverse*motorPort.getCurrentPosition();
    }

    public double getTicks() {
        return getRawTicks() * reduction;
    }

    public double getDistance() {
        return getTicks() * conversionFactor;
    }

    public void reset() {
        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum ConversionFactor {
        DEGREE,
        RADIAN,
        REVOLUTIONS
    }
}
