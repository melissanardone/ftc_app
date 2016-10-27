package org.steelhead.ftc;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Alec Matthews on 10/23/2016.
 * This class is a PID controller built for a color sensor.
 */

public class ColorPIDController {
    //initialize values
    private Thread pidThread;
    private int offsetValue;
    private boolean isOutputAvailable = false;
    private double output;
    private boolean isActive = true;
    private double kp;
    private double ki;
    private double kd;
    private ElapsedTime runtime;

    public ColorPIDController(final ColorSensor colorSensor, int thresholdLow, int thresholdHigh) {
        this.offsetValue = (thresholdLow + thresholdHigh)/2;

        //Setup the separate thread for calculating the values
        //This is in a separate thread so it doesn't slow down the main thead.
        pidThread = new Thread(new Runnable() {
            @Override
            public void run() {
                //Calculate the PID output
                double error;
                double lastError = 0;
                double integral = 0;
                double derivative;
                while (isActive) {
                    error = colorSensor.alpha() - offsetValue;
                    integral = integral + error;
                    derivative = error - lastError;
                    output = (kp * error) + (ki * integral) + (kd * derivative);
                    lastError = error;
                }
            }
        });
    }
    //This function is used to set the PID values. It must be run before the thread is started
    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    //Start the thread
    public void enable() {
        pidThread.start();
    }

    //Stop the thread
    public void disable() {
        isActive = false;
    }
    @Deprecated
    public boolean isOutputAvailable() {
        return isOutputAvailable;
    }
    //Get the value of calculated by the PID controller
    public double getOutput() {
        return output;
    }
}
