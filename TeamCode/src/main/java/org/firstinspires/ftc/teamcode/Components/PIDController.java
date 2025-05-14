package org.firstinspires.ftc.teamcode.Components;

public class PIDController {
    private double kp, ki, kd;
    public double previousError;
    private double integral;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.previousError = 0.0;
        this.integral = 0.0;
    }

    public double update(double error, double deltaTime) {
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        return kp * error + ki * integral + kd * derivative;
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

