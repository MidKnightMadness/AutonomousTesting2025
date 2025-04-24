package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Components.Sample;

@Config
@TeleOp(name = "VisionTest")
public class VisionTest extends OpMode {
    public static double BUFFER_TIME = 0.5;//Seconds
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            vision.update();
//            //Get closest sample of fov
            Sample sampleDetected = vision.getClosestSample(SampleColors.YELLOW, BUFFER_TIME);
            telemetry.addLine(sampleDetected.toString());
        }

    }
}
