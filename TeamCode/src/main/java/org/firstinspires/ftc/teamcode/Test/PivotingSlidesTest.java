package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;


@TeleOp(group="Test")
public class PivotingSlidesTest extends OpMode {

    PivotingSlides pivotingSlides;
    Timer timer;

    @Override
    public void init() {
        pivotingSlides = new PivotingSlides(hardwareMap);
        timer = new Timer();
    }

    double extensionLength = 9;

    @Override
    public void loop() {
        timer.updateTime();
        double deltaTime = timer.getDeltaTime();

        extensionLength += gamepad1.dpad_up ? deltaTime * 1 : 0;
        extensionLength -= gamepad1.dpad_down ? deltaTime * 1 : 0;

        if (gamepad1.a) pivotingSlides.setExtensionLength(extensionLength);
        if (gamepad1.b) pivotingSlides.setExtensionLength(0);

        telemetry.addData("Extension length", pivotingSlides.getExtensionLength());
        telemetry.update();
    }
}
