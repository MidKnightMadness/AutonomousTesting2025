package org.firstinspires.ftc.teamcode.ColorSensor;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorClassifier {

    public static class LedOn {
        public static RGB yellowLowerBound = new RGB(0.3, 0.4, 0);
        public static RGB yellowUpperBound = new RGB(1, 1, 0.3);

        public static RGB redLowerBound = new RGB(0.4, 0, 0);
        public static RGB redUpperBound = new RGB(1, 0.4, 0.4);

        public static RGB blueLowerBound = new RGB(0, 0, 0.4);
        public static RGB blueUpperBound = new RGB(0.4, 0.4, 1);
    }

    public static class LedOff {
        public static RGB yellowLowerBound = new RGB(0.3, 0.3, 0);
        public static RGB yellowUpperBound = new RGB(0.6, 0.5, 0.15);

        public static RGB redLowerBound = new RGB(0.4, 0, 0);
        public static RGB redUpperBound = new RGB(1, 0.3, 0.3);

        public static RGB blueLowerBound = new RGB(0.25, 0.25, 0.25);
        public static RGB blueUpperBound = new RGB(0.4, 0.4, 1);
    }



    public static boolean isBetween(RGB target, RGB lower, RGB upper){
        if (target.r < lower.r || target.r > upper.r) return false;
        if (target.g < lower.g || target.g > upper.g) return false;
        if (target.b < lower.b || target.b > upper.b) return false;

        return true;
    }

    public static SampleColors classifyLedOn(RGB color) {
        color.normalize();

        if (isBetween(color, LedOn.yellowLowerBound, LedOn.yellowUpperBound)) return SampleColors.YELLOW;
        if (isBetween(color, LedOn.redLowerBound, LedOn.redUpperBound)) return SampleColors.RED;
        if (isBetween(color, LedOn.blueLowerBound, LedOn.blueUpperBound)) return SampleColors.BLUE;

        return SampleColors.NONE;
    }

    public static SampleColors classifyLedOff(RGB color) {
        color.normalize();

        if (isBetween(color, LedOff.yellowLowerBound, LedOff.yellowUpperBound)) return SampleColors.YELLOW;
        if (isBetween(color, LedOff.redLowerBound, LedOff.redUpperBound)) return SampleColors.RED;
        if (isBetween(color, LedOff.blueLowerBound, LedOff.blueUpperBound)) return SampleColors.BLUE;

        return SampleColors.NONE;
    }
}
