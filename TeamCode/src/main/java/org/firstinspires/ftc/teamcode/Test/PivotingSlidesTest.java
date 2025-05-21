package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Mechanisms.PivotingSlides;


@TeleOp(group="Test")
@Config
public class PivotingSlidesTest extends OpMode {

    PivotingSlides pivotingSlides;
    Timer timer;

    @Override
    public void init() {
        pivotingSlides = new PivotingSlides(hardwareMap);
        timer = new Timer();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public static double extensionLength = 9;
    double speed = 1;

    @Override
    public void loop() {
        timer.updateTime();

        double deltaTime = timer.getDeltaTime();

        extensionLength += gamepad1.dpad_up ? deltaTime * speed : 0;
        extensionLength -= gamepad1.dpad_down ? deltaTime * speed : 0;

        pivotingSlides.setExtension(extensionLength);


        telemetry.addData("Extension length", extensionLength);
        telemetry.addData("Servo angle", pivotingSlides.targetAngle);
        telemetry.addData("Servo position", pivotingSlides.targetPos);
        telemetry.update();
    }
}
