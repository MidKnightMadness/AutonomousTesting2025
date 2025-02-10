package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorTest extends OpMode {
    RevColorSensorV3 colorSensorV3;

    @Override
    public void init() {
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    @Override
    public void loop() {
        NormalizedRGBA color = colorSensorV3.getNormalizedColors();
        double dist = colorSensorV3.getDistance(DistanceUnit.INCH);

        telemetry.addData("color", color);
        telemetry.addData("dist", dist);
    }
}
