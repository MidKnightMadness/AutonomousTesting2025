package org.firstinspires.ftc.teamcode.Components;

public class Util {

    public static double clamp(double value, double min, double max) {
        if (value > max) return max;
        if (value < min) return min;

        return value;
    }
    public static int clamp(int value, int min, int max) {
        if (value > max) return max;
        if (value < min) return min;

        return value;
    }
}
