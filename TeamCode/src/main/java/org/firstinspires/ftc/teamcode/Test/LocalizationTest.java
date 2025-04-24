package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Localization.GoBildaPinpoint.PinpointOdometryLocalizer;
import org.firstinspires.ftc.teamcode.Localization.messages.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="LocalizationTest", group="Test")
@Config
public class LocalizationTest extends LinearOpMode {

//    Localizer otosLocalizer;
    PinpointOdometryLocalizer pinpointLocalizer;

    Timer timer;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startingPose = new Pose2d(0, 0, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose, telemetry);

        pinpointLocalizer = new PinpointOdometryLocalizer(hardwareMap, startingPose);
        timer = new Timer();

        waitForStart();

        while (opModeIsActive()) {
            timer.updateTime();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_x ,
                            -gamepad1.left_stick_y
                    ),
                    -gamepad1.right_stick_x * 0.5
            ));

            drive.updatePoseEstimate();
            pinpointLocalizer.update();

            telemetry.addData("Update rate (Hz)", 1 / timer.getDeltaTime());
            telemetry.addLine();

            Pose2d pose = drive.localizer.getPose();
            telemetry.addLine("\nDrive Localizer (OTOS)");
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d pose2 = pinpointLocalizer.getPose();
            telemetry.addLine("\nDrive Localizer (Pinpoint)");
            telemetry.addData("x", pose2.position.x);
            telemetry.addData("y", pose2.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose2.heading.toDouble()));


            telemetry.addLine("\nRaw IMU Values");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
