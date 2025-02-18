package org.firstinspires.ftc.teamcode.Color;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorClassifier {

    // TODO
    public static RGB yellowLowerBound = new RGB(0.5, 0.5, 0);
    public static RGB yellowUpperBound = new RGB(1, 1, 0.3);

    public static RGB redLowerBound = new RGB(0.5, 0, 0);
    public static RGB redUpperBound = new RGB(1, 0.3, 0.3);

    public static RGB blueLowerBound = new RGB(0, 0, 0.5);;
    public static RGB blueUpperBound = new RGB(0.3, 0.3, 1);;

    public static boolean isBetween(RGB target, RGB lower, RGB upper){
        if (target.r > lower.r || target.r < upper.r) return false;
        if (target.g > lower.g || target.g < upper.g) return false;
        if (target.b > lower.b || target.b < upper.b) return false;

        return true;
    }

    public static SampleColors classify(RGB color) {
        color.normalize();

        if (isBetween(color, yellowLowerBound, yellowUpperBound)) return SampleColors.YELLOW;
        if (isBetween(color, redLowerBound, redUpperBound)) return SampleColors.RED;
        if (isBetween(color, blueLowerBound, blueUpperBound)) return SampleColors.BLUE;

        return SampleColors.NONE;
    }
}
