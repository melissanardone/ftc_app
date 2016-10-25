package org.steelhead.ftc;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Alec Matthews on 10/23/2016.
 */

public class ColorPIDController implements Runnable {
    private ColorSensor colorSensor;
    private int offsetValue;
    private boolean isOutputAvailable = false;
    private double output;
    private boolean isActive = true;
    private double kp;
    private double ki;
    private double kd;

    public ColorPIDController(ColorSensor colorSensor, int thresholdLow, int thresholdHigh) {
        this.colorSensor = colorSensor;
        this.offsetValue = (thresholdLow + thresholdHigh)/2;
    }
    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void stop() {
        isActive = false;
    }
    public boolean isOutputAvailable() {
        return isOutputAvailable;
    }
    public double getOutput() {
        return output;
    }
    @Override
    public void run() {
        double error;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        while (isActive) {
            //TODO: implement the pid controller for the color sensor READ ARTICLE!!
            //TODO: test color sensor values to see which one gives the best reading
            error = colorSensor.alpha() - offsetValue;
            integral = integral + error;
            derivative = error - lastError;
            output = (kp*error) + (ki*integral) + (kd*derivative);
            lastError = error;
        }
    }
}
