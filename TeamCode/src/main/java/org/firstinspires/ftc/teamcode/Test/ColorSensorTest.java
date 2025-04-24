package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.RGB;

@TeleOp(group="Test")
public class ColorSensorTest extends OpMode {
    RevColorSensorV3 colorSensorV3;


    @Override
    public void init() {
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    @Override
    public void loop() {
        NormalizedRGBA color = colorSensorV3.getNormalizedColors();

        RGB colorRGB = new RGB(color);

        telemetry.addData("color value",colorRGB);
        telemetry.addData("Classifier (Led on)", ColorClassifier.classifyLedOn(colorRGB));
        telemetry.addData("Classifier (Led off)", ColorClassifier.classifyLedOff(colorRGB));

        telemetry.addData("dist", colorSensorV3.getDistance(DistanceUnit.INCH));
        telemetry.addData("light detected", colorSensorV3.getLightDetected());
    }
}
