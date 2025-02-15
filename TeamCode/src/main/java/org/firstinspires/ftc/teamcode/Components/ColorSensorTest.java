package org.firstinspires.ftc.teamcode.Components;

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

        double dist = colorSensorV3.getDistance(DistanceUnit.INCH);

        double sum = color.red + color.green + color.blue;
        telemetry.addData("r",color.red / sum);
        telemetry.addData("g",color.green / sum);
        telemetry.addData("b",color.blue / sum);
        telemetry.addData("a",color.alpha);

        telemetry.addData("dist", dist);
        telemetry.addData("light detected", colorSensorV3.getLightDetected());
        telemetry.addData("raw light detected", colorSensorV3.getRawLightDetected());
        telemetry.addData("", colorSensorV3.rawOptical());
    }
}
