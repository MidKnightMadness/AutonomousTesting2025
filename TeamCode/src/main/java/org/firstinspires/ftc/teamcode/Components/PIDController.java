package org.firstinspires.ftc.teamcode.Components;

public class PIDController {
    private double kp, ki, kd;
    public double previousError;
    private double integral;

    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.previousError = 0.0;
        this.integral = 0.0;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public double update(double error, double deltaTime) {
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        double output = kp * error + ki * integral + kd * derivative;
//
//        output = Math.max(minOutput, Math.min(output, maxOutput));
//
//        // Prevent integral windup
//        if (output == minOutput || output == maxOutput) {
//            integral -= error * deltaTime;
//        }

        return output;
    }

    public void reset() {
        previousError = 0.0;
        integral = 0.0;
    }

    public void setCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}

