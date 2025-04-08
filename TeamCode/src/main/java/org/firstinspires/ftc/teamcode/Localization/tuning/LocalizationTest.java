package org.firstinspires.ftc.teamcode.Localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Timer;
import org.firstinspires.ftc.teamcode.Localization.GoBildaPinpoint.PinpointOdometryLocalizer;
import org.firstinspires.ftc.teamcode.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Localization.TwoDeadWheelOTOSLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RunOptions;

import java.util.List;

@TeleOp(name="LocalizationTest")
@Config
public class LocalizationTest extends LinearOpMode {

//    Localizer otosLocalizer;
    PinpointOdometryLocalizer pinpointLocalizer;

    Timer timer;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startingPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose, telemetry);
        drive.otos.calibrateImu();
        drive.otos.resetTracking();

        pinpointLocalizer = new PinpointOdometryLocalizer(hardwareMap, telemetry);
        timer = new Timer();

//        otosLocalizer = new TwoDeadWheelOTOSLocalizer(hardwareMap, drive.otos, MecanumDrive.PARAMS.inPerTick, startingPose);

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

            if (this.gamepad1.y) {
                drive.otos.calibrateImu();
                drive.otos.resetTracking();
            }

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


            telemetry.addLine("\nDrive Localizer (Pinpoint2)");
            telemetry.addData("x", pinpointLocalizer.odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("y", pinpointLocalizer.odo.getPosX());
            telemetry.addData("heading (deg)", pinpointLocalizer.odo.getHeading());


            telemetry.addLine("\nRaw IMU Values");
            telemetry.addData("heading (OTOS)", Math.toDegrees(drive.otos.getPosition().h));

            telemetry.update();
        }
    }
}
