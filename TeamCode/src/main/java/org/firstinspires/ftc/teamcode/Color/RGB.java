package org.firstinspires.ftc.teamcode.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class RGB {
    public double r;
    public double g;
    public double b;

    public RGB() { }

    public RGB(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public RGB(NormalizedRGBA color) {
        this.r = color.red;
        this.g = color.green;
        this.b = color.blue;
    }

    public RGB normalize() {
        double sum = r + g + b;
        this.r /= sum;
        this.g /= sum;
        this.b /= sum;

        return this;
    }
}
