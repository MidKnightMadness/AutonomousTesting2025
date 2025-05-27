package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TouchTest extends OpMode {

    TouchSensor touchSensor;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(RevTouchSensor.class, "Arm Touch");
    }

    @Override
    public void loop() {
        telemetry.addData("Ispressed", touchSensor.isPressed());
        telemetry.addData("value", touchSensor.getValue());
    }

}
