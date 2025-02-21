package org.firstinspires.ftc.teamcode.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class ColorSensorTest extends OpMode {
    RevColorSensorV3 colorSensorV3;
    public static boolean useLed = true;


    @Override
    public void init() {
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Claw Color Sensor");
    }

    @Override
    public void loop() {
        NormalizedRGBA color = colorSensorV3.getNormalizedColors();
        colorSensorV3.enableLed(useLed);

        RGB colorRGB = new RGB(color);

        telemetry.addData("color value",colorRGB);
        telemetry.addData("Classifier",ColorClassifier.classify(colorRGB));

        telemetry.addData("dist", colorSensorV3.getDistance(DistanceUnit.INCH));
        telemetry.addData("light detected", colorSensorV3.getLightDetected());
    }
}
