package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;

@TeleOp(group="Test")
@Config
public class ArmTest extends OpMode {

    Arm arm;

    public static double positionDirect = 0.1;
    public static double degreesDirect = 0;

    public static boolean useDegrees = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (useDegrees) {
            arm.setPosition(Arm.degreesToPosition(degreesDirect));
            return;
        }
        arm.setPosition(positionDirect);
    }
}
