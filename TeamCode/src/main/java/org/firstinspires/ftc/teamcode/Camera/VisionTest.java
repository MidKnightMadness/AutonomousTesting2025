package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "visionTest", group = "Vision")
public class VisionTest extends OpMode {
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            vision.update();
        }

    }
}
