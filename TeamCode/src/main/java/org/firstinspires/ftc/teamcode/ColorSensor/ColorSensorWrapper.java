package org.firstinspires.ftc.teamcode.ColorSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorWrapper {
    RevColorSensorV3 colorSensor;
    SampleColors allianceColor;
    public static double SAMPLE_PICKUP_THRESHOLD = 2; // inches

    public static class ColorSensorReading {
        public boolean isValid;
        public RGB color;
        public double distance;

        public ColorSensorReading(NormalizedRGBA color, double distance) {
            isValid = !(Double.isNaN(color.blue) || Double.isNaN(color.red) || Double.isNaN(color.green));
            this.color = new RGB(color);
            this.distance = distance;
        }
    }

    public ColorSensorWrapper(HardwareMap hardwareMap, SampleColors allianceColor) {
        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, "Claw Color Sensor");
        this.allianceColor = allianceColor;
    }

    public ColorSensorReading read() {
        return new ColorSensorReading(colorSensor.getNormalizedColors(), colorSensor.getDistance(DistanceUnit.INCH));
    }

    public boolean isCorrectColor(int numChecks) {
        for (int i = 0; i < numChecks; i++) {
            ColorSensorReading reading = read();
            if (!reading.isValid) return true;  // color sensor disconnect, don't check for color

            SampleColors color = ColorClassifier.classifyLedOn(reading.color);
            if (color == SampleColors.YELLOW || color == allianceColor) {
                return true;
            }
        }

        return false;
    }

    public boolean isSamplePickedUp(int numChecks) {
        for (int i = 0; i < numChecks; i++) {
            ColorSensorReading reading = read();
            if (!reading.isValid) return true;  // color sensor disconnect, don't check for color

            SampleColors color = ColorClassifier.classifyLedOn(reading.color);
            if (color == SampleColors.YELLOW || color == allianceColor && colorSensor.getDistance(DistanceUnit.INCH) < SAMPLE_PICKUP_THRESHOLD) {
                return true;
            }
        }

        return false;
    }
}
