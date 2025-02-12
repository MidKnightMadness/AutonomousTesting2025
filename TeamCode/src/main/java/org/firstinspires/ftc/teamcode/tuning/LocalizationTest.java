package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelIMULocalizer;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@TeleOp(name="LocalizationTest2")
public class LocalizationTest extends LinearOpMode {

    IMU imuControl;
    IMU imuExpansion;
    Localizer firstLocalizer;
    Localizer secondLocalizer;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startingPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        imuControl = hardwareMap.get(IMU.class, "imuControl");
        imuControl.resetYaw();
        imuExpansion = hardwareMap.get(IMU.class, "imuExpansion");
        imuExpansion.resetYaw();

        firstLocalizer = new ThreeDeadWheelIMULocalizer(hardwareMap, imuControl, MecanumDrive.PARAMS.inPerTick, startingPose);
        secondLocalizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, startingPose);


        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();
            firstLocalizer.update();
            secondLocalizer.update();

            Pose2d pose = firstLocalizer.getPose();


            telemetry.addLine("\nFirst Localizer");
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));


            Pose2d pose2 = secondLocalizer.getPose();

            telemetry.addLine("\nSecond Localizer");
            telemetry.addData("x", pose2.position.x);
            telemetry.addData("y", pose2.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose2.heading.toDouble()));

            telemetry.addLine("\nIMU Localizer");
            telemetry.addData("heading (control)", (imuControl.getRobotYawPitchRollAngles().getYaw()));
            telemetry.addData("heading (expansion)", (imuExpansion.getRobotYawPitchRollAngles().getYaw()));


            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
